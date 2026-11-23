#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "controller_utils.hpp"
#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"
#include "utils.hpp"

std::vector<std::string> default_joint_names() {
    return {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7"};
}

JointArray to_joint_array(const Eigen::VectorXd& value) {
    if (value.size() != 7) {
        throw std::runtime_error("expected a 7-element joint vector");
    }
    JointArray out{};
    for (std::size_t j = 0; j < 7; ++j) out[j] = value[static_cast<Eigen::Index>(j)];
    return out;
}

bool all_finite(const JointArray& q) {
    return std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); });
}

double max_abs(const JointArray& q) {
    double value = 0.0;
    for (double v : q) value = std::max(value, std::abs(v));
    return value;
}

JointArray subtract(const JointArray& a, const JointArray& b) {
    JointArray result{};
    for (std::size_t i = 0; i < 7; ++i) result[i] = a[i] - b[i];
    return result;
}

JointArray add_scaled(const JointArray& base, const JointArray& delta, double scale) {
    JointArray result{};
    for (std::size_t i = 0; i < 7; ++i) result[i] = base[i] + scale * delta[i];
    return result;
}

JointArray scaled(const JointArray& value, double scale) {
    JointArray result{};
    for (std::size_t i = 0; i < 7; ++i) result[i] = value[i] * scale;
    return result;
}

void set_point_duration(trajectory_msgs::msg::JointTrajectoryPoint& point,
                        double seconds) {
    const int64_t nanoseconds = static_cast<int64_t>(std::llround(seconds * 1.0e9));
    point.time_from_start.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
}

void append_point(trajectory_msgs::msg::JointTrajectory& trajectory,
                  double time_from_start, const JointArray& positions,
                  const JointArray& velocities, const JointArray& accelerations) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(positions.begin(), positions.end());
    point.velocities.assign(velocities.begin(), velocities.end());
    point.accelerations.assign(accelerations.begin(), accelerations.end());
    set_point_duration(point, time_from_start);
    trajectory.points.push_back(std::move(point));
}

double last_point_time(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (trajectory.points.empty()) return -1.0;
    const auto& last = trajectory.points.back().time_from_start;
    return static_cast<double>(last.sec) + 1.0e-9 * last.nanosec;
}


double max_abs_difference(const JointArray& a, const JointArray& b) {
    double value = 0.0;
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        value = std::max(value, std::abs(a[j] - b[j]));
    }
    return value;
}


void append_waypoint(trajectory_msgs::msg::JointTrajectory& trajectory,
                     double time_from_start, const Waypoint& waypoint) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(waypoint.q.data(), waypoint.q.data() + waypoint.q.size());
    point.velocities.assign(waypoint.qd.data(), waypoint.qd.data() + waypoint.qd.size());
    point.accelerations.assign(waypoint.qdd.data(),
                               waypoint.qdd.data() + waypoint.qdd.size());
    set_point_duration(point, time_from_start);
    trajectory.points.push_back(std::move(point));
}


void append_quintic(trajectory_msgs::msg::JointTrajectory& trajectory,
                    double t0, double duration, double dt,
                    const JointArray& p0, const JointArray& v0, const JointArray& a0,
                    const JointArray& p1, const JointArray& v1, const JointArray& a1) {
    if (!(duration > 0.0)) return;
    const std::size_t count = static_cast<std::size_t>(
        std::max(1.0, std::ceil(duration / dt)));

    std::array<JointArray, 6> c{};
    const double t1 = duration;
    const double t2 = duration * duration;
    for (std::size_t j = 0; j < 7; ++j) {
        const double dp = p1[j] - p0[j];
        c[0][j] = p0[j];
        c[1][j] = v0[j];
        c[2][j] = 0.5 * a0[j];
        c[3][j] = (20.0 * dp - (8.0 * v1[j] + 12.0 * v0[j]) * t1 -
                   (3.0 * a0[j] - a1[j]) * t2) /
                  (2.0 * t1 * t2);
        c[4][j] = (-30.0 * dp + (14.0 * v1[j] + 16.0 * v0[j]) * t1 +
                   (3.0 * a0[j] - 2.0 * a1[j]) * t2) /
                  (2.0 * t2 * t2);
        c[5][j] = (12.0 * dp - 6.0 * (v1[j] + v0[j]) * t1 -
                   (a0[j] - a1[j]) * t2) /
                  (2.0 * t2 * t2 * t1);
    }

    for (std::size_t k = 0; k < count; ++k) {
        const double t = duration * static_cast<double>(k) /
                         static_cast<double>(count);
        JointArray p{}, v{}, a{};
        for (std::size_t j = 0; j < 7; ++j) {
            p[j] = c[0][j] + t * (c[1][j] + t * (c[2][j] + t * (c[3][j] +
                   t * (c[4][j] + t * c[5][j]))));
            v[j] = c[1][j] + t * (2.0 * c[2][j] + t * (3.0 * c[3][j] +
                   t * (4.0 * c[4][j] + t * 5.0 * c[5][j])));
            a[j] = 2.0 * c[2][j] + t * (6.0 * c[3][j] + t * (12.0 * c[4][j] +
                   t * 20.0 * c[5][j]));
        }
        append_point(trajectory, t0 + t, p, v, a);
    }
}


Waypoint receiveRobotStates(DataTransmitter& transmitter) {
    const std::vector<nlohmann::json> data = transmitter.receive_data();
    if (data.size() < 5) return {};

    Waypoint wp;
    wp.q = json_to_robotstate(data[2]);
    wp.qd = json_to_robotstate(data[3]);
    wp.qdd = json_to_robotstate(data[4]);
    return wp;
}


std::string result_code_to_string(int32_t code) {
    switch (code) {
        case FollowJointTrajectory::Result::SUCCESSFUL:
            return "SUCCESSFUL";
        case FollowJointTrajectory::Result::INVALID_GOAL:
            return "INVALID_GOAL";
        case FollowJointTrajectory::Result::INVALID_JOINTS:
            return "INVALID_JOINTS";
        case FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP";
        case FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED";
        case FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED";
        default:
            return "UNKNOWN(" + std::to_string(code) + ")";
    }
}

