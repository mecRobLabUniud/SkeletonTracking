#pragma once

/*
░█▀▄░█▀█░▀█▀░█▀█░░░▀█▀░█▀▄░█▀█░█▀█░█▀▀░█▄█░▀█▀░▀█▀░▀█▀░█▀▀░█▀▄
░█░█░█▀█░░█░░█▀█░░░░█░░█▀▄░█▀█░█░█░▀▀█░█░█░░█░░░█░░░█░░█▀▀░█▀▄
░▀▀░░▀░▀░░▀░░▀░▀░░░░▀░░▀░▀░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░░▀░░░▀░░▀▀▀░▀░▀

Trajectory publication: replaces the libfranka robot handle.

Every trajectory sample that used to be streamed into libfranka's 1 kHz
robot.control() callback is instead published through DataTransmitter
(data_transmitter.hpp — the C++ mirror of utils/data_transmitter.py) to
web_interface.py, which subscribes to (in --robot mode):
  - "ROBOT"    (device_id 12): [robot_points, joint_positions, radii]
  - "TRAJDATA" (device_id 14): [closest_human_point, closest_robot_point,
                                collision_flag]
*/

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include "data_transmitter.hpp"
#include "flag_stop.h"  // for Capsule / computeRobotCapsules

// Playback period: matches the old libfranka fixed 1kHz control period
constexpr double TRAJ_STEP = 0.001;
// Maximum torque rate, mirrors franka::kMaxTorqueRate (Nm/s)
constexpr double kMaxTorqueRate = 1000.0;
// How often the commanded trajectory is published to web_interface.py (s).
// Mirrors the 30 Hz rate used by SkeletonTracking's exec_trajectory.cpp.
constexpr double TRAJ_PUBLISH_PERIOD = 1.0 / 30.0;


// ─────────────────────────────────────────────────────────────────────────────
// Stand-in for franka::RobotState inside the 1 kHz playback loop: the
// simulated plant state. There is no robot attached, so the "measured" state
// simply tracks the commanded trajectory (one tick of delay, like the real
// robot).
// ─────────────────────────────────────────────────────────────────────────────
struct PlaybackState {
    std::array<double, 7> q{};        // joint positions (measured)
    std::array<double, 7> dq{};       // joint velocities (measured)
    std::array<double, 7> ddq_d{};    // desired joint accelerations
    std::array<double, 7> tau_J_d{};  // last commanded torque (rate-limit reference)
    std::array<double, 7> tau_J{};    // "measured" torque
};


// ─────────────────────────────────────────────────────────────────────────────
// Rate-limit each torque command around the previously commanded torque,
// mirroring franka::limitRate(franka::kMaxTorqueRate, tau_cmd, state.tau_J_d)
// ─────────────────────────────────────────────────────────────────────────────
inline std::array<double, 7> limitTorqueRate(
    const std::array<double, 7>& tau_cmd,
    const std::array<double, 7>& tau_last)
{
    const double max_delta = kMaxTorqueRate * TRAJ_STEP;
    std::array<double, 7> out;
    for (int j = 0; j < 7; ++j) {
        out[j] = tau_last[j] + std::clamp(tau_cmd[j] - tau_last[j], -max_delta, max_delta);
    }
    return out;
}


// ─────────────────────────────────────────────────────────────────────────────
// JSON helpers. nlohmann::json cannot serialize NaN, while Python's json.dumps
// (on the data_merging.py side) emits the bare token "NaN". Map NaN to null
// here so the Python receiver's json.loads reproduces the same "missing value"
// semantics as the rest of the pipeline.
// ─────────────────────────────────────────────────────────────────────────────
inline nlohmann::json jsonNumber(double v) {
    return std::isnan(v) ? nlohmann::json(nullptr) : nlohmann::json(v);
}

inline nlohmann::json jsonPoint(const Eigen::Vector3d& p) {
    return nlohmann::json::array({jsonNumber(p.x()), jsonNumber(p.y()), jsonNumber(p.z())});
}

inline nlohmann::json jsonPoint(const std::array<double, 3>& p) {
    return nlohmann::json::array({jsonNumber(p[0]), jsonNumber(p[1]), jsonNumber(p[2])});
}


// ─────────────────────────────────────────────────────────────────────────────
// Replaces the libfranka robot handle: streams the trajectory to web_interface.py
// ─────────────────────────────────────────────────────────────────────────────
class TrajectoryPublisher {
public:
    TrajectoryPublisher()
        : robot_tx_(DataTransmitter::Mode::Sender, 12, "ROBOT"),
          trajdata_tx_(DataTransmitter::Mode::Sender, 14, "TRAJDATA") {}

    // Publish one trajectory sample: robot capsule endpoints, joint positions
    // and capsule radii (robot_p / robot_q / caps_radius expected by
    // web_interface.py in --robot mode)
    void publishSample(const pinocchio::Model& model,
                       pinocchio::Data&        data,
                       const Eigen::VectorXd&  q) {
        std::array<Capsule, 4> capsules = computeRobotCapsules(model, data, q);

        nlohmann::json points = nlohmann::json::array();
        for (const auto& c : capsules) {
            points.push_back(jsonPoint(c.A));
            points.push_back(jsonPoint(c.B));
        }

        nlohmann::json q_json = nlohmann::json::array();
        for (int j = 0; j < 7; ++j) {
            q_json.push_back(jsonNumber(q(j)));
        }

        nlohmann::json radii = nlohmann::json::array();
        for (const auto& c : capsules) {
            radii.push_back(c.radius);
        }

        robot_tx_.send_data({points, q_json, radii});
    }

    // Publish the closest human/robot point pair and the stop decision
    // (p_real / p_r / collision expected by web_interface.py)
    void publishClosestPoints(const Eigen::Vector3d& C_h,
                              const Eigen::Vector3d& C_r,
                              bool                    collision) {
        trajdata_tx_.send_data({jsonPoint(C_h), jsonPoint(C_r),
                                nlohmann::json(collision ? 1 : 0)});
    }

    void shutdown() {
        robot_tx_.shutdown();
        trajdata_tx_.shutdown();
    }

private:
    DataTransmitter robot_tx_;
    DataTransmitter trajdata_tx_;
};
