/*
robot_controller_ros2.cpp - ROS2 (multipanda_ros2) port of robot_controller.cpp.

Replaces libfranka calls with the ROS2 FollowJointTrajectory action interface.
Receives the ROBOT_12 stream over ZMQ, builds point-to-point trajectory samples,
and sends them as FollowJointTrajectory goals to the joint_trajectory_controller.

Usage:
  robot_controller_ros2 <traj_index> [repo_dir] [--ros-args -p name:=value]
*/

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <Eigen/Dense>

#include "data_transmitter.hpp"
#include "project_root.hpp"
#include "robot_model.hpp"
#include "trajectory_utils.hpp"
#include "utils.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using JointArray = std::array<double, 7>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

constexpr double kRateHz = 60.0;
constexpr double kStreamDt = 1.0 / kRateHz;
constexpr double kTrajectoryDt = 0.001;
constexpr int kRobotZmqDevice = 12;
constexpr int kZmqBasePort = 6000;

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

}  // namespace

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode(int n_traj, const std::filesystem::path& root)
        : Node("robot_controller_ros2"), n_traj_(n_traj), root_(root) {
        declare_parameters();
        validate_parameters();

        const std::filesystem::path urdf = root_ / "src" / "urdf" / "panda.urdf";
        if (!std::filesystem::exists(urdf)) {
            throw std::runtime_error("URDF not found: " + urdf.string());
        }
        robot_model_ = std::make_unique<RobotModel>(urdf.string());

        nominal_traj_ = load_trajectory(*robot_model_, root_.string() + "/", n_traj_, kStreamDt);
        if (nominal_traj_.wp.empty()) {
            throw std::runtime_error("the nominal trajectory is empty");
        }

        joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, rclcpp::QoS(10),
            std::bind(&RobotControllerNode::joint_state_callback, this,
                      std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, controller_action_);

        send_service_ = create_service<std_srvs::srv::Trigger>(
            "~/send",
            std::bind(&RobotControllerNode::send_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        cancel_service_ = create_service<std_srvs::srv::Trigger>(
            "~/cancel",
            std::bind(&RobotControllerNode::cancel_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        receiver_thread_ = std::thread(&RobotControllerNode::receiver_loop, this);

        poll_timer_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RobotControllerNode::poll_callback, this));
    }

    ~RobotControllerNode() override {
        stop_receiver_.store(true);
        if (receiver_thread_.joinable()) receiver_thread_.join();
    }

private:
    void declare_parameters() {
        controller_action_ = declare_parameter<std::string>(
            "controller_action", "/panda_effort_jtc/follow_joint_trajectory");
        joint_state_topic_ = declare_parameter<std::string>(
            "joint_state_topic", "/joint_states");
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", default_joint_names());
        dry_run_ = declare_parameter<bool>("dry_run", false);
        stream_timeout_ = declare_parameter<double>("stream_timeout", 1.0);
        approach_duration_ = declare_parameter<double>("approach_duration", 3.0);
        approach_dt_ = declare_parameter<double>("approach_dt", 0.01);
        max_approach_distance_ = declare_parameter<double>("max_approach_distance", 0.5);
        start_tolerance_ = declare_parameter<double>("start_tolerance", 0.15);

        streaming_ = declare_parameter<bool>("streaming", false);
        chunk_duration_ = declare_parameter<double>("chunk_duration", 1.0);
        chunk_lead_ = declare_parameter<double>("chunk_lead", 0.3);
        chunk_ramp_ = declare_parameter<double>("chunk_ramp", 0.2);
        min_chunk_duration_ = declare_parameter<double>("min_chunk_duration", 0.3);
        max_chunk_step_ = declare_parameter<double>("max_chunk_step", 0.35);
        max_ramp_velocity_ = declare_parameter<double>("max_ramp_velocity", 1.0);
    }

    void validate_parameters() {
        if (joint_names_.size() != 7) {
            throw std::runtime_error("joint_names must contain exactly 7 names");
        }
        if (!(stream_timeout_ > 0.0)) {
            throw std::runtime_error("stream_timeout must be greater than zero");
        }
        if (approach_duration_ < 0.0) {
            throw std::runtime_error("approach_duration must be zero or positive");
        }
        if (!(approach_dt_ > 0.0)) {
            throw std::runtime_error("approach_dt must be greater than zero");
        }
        if (max_approach_distance_ < 0.0) {
            throw std::runtime_error("max_approach_distance must be zero or positive");
        }
        if (start_tolerance_ < 0.0) {
            throw std::runtime_error("start_tolerance must be zero or positive");
        }
        if (streaming_) {
            if (!(chunk_duration_ > 0.05)) {
                throw std::runtime_error("chunk_duration must be greater than 0.05 s");
            }
            if (chunk_lead_ < 0.0 || chunk_lead_ >= chunk_duration_) {
                throw std::runtime_error(
                    "chunk_lead must be zero or positive and smaller than chunk_duration");
            }
            if (chunk_ramp_ < 0.0 || chunk_ramp_ >= chunk_duration_) {
                throw std::runtime_error(
                    "chunk_ramp must be zero or positive and smaller than chunk_duration");
            }
            if (!(min_chunk_duration_ > 0.0) ||
                min_chunk_duration_ > chunk_duration_) {
                throw std::runtime_error(
                    "min_chunk_duration must be positive and at most chunk_duration");
            }
            if (!(max_chunk_step_ > 0.0)) {
                throw std::runtime_error("max_chunk_step must be greater than zero");
            }
            if (!(max_ramp_velocity_ > 0.0)) {
                throw std::runtime_error("max_ramp_velocity must be greater than zero");
            }
        }
    }

    void receiver_loop() {
        DataTransmitter receiver(DataTransmitter::Mode::Receiver, kRobotZmqDevice, "ROBOT");
        receiver.set_receive_timeout(20);

        Waypoint previous = nominal_traj_.wp.front();
        Waypoint next = previous;
        double t = 0.0;
        std::vector<double> vnorm;
        bool received_message = false;
        Clock::time_point last_message = Clock::now();

        while (!stop_receiver_.load() && rclcpp::ok()) {
            Waypoint wp;
            try {
                wp = receiveRobotStates(receiver);
            } catch (const std::exception&) {
                continue;
            }

            if (wp.q.size() == 7 && wp.qd.size() == 7 && wp.qdd.size() == 7 &&
                wp.q.allFinite() && wp.qd.allFinite() && wp.qdd.allFinite()) {
                received_message = true;
                last_message = Clock::now();

                if (wp.q != next.q) {
                    std::lock_guard<std::mutex> lock(trajectory_mutex_);
                    previous = next;
                    next = wp;
                    build_p2p_traj(previous.q, next.q, &vnorm, real_traj_, kStreamDt, t,
                                   *robot_model_, kTrajectoryDt, previous.qd,
                                   previous.qdd, next.qd, next.qdd);
                }
            }

            if (received_message &&
                std::chrono::duration<double>(Clock::now() - last_message).count() >
                    stream_timeout_) {
                break;
            }
        }

        std::size_t samples = 0;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            samples = real_traj_.wp.size();
            stream_finished_.store(true);
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message) {
        JointArray positions{};
        JointArray velocities{};
        for (std::size_t j = 0; j < 7; ++j) {
            const auto iterator =
                std::find(message->name.begin(), message->name.end(), joint_names_[j]);
            if (iterator == message->name.end()) return;
            const auto index = static_cast<std::size_t>(
                std::distance(message->name.begin(), iterator));
            if (index >= message->position.size()) return;
            positions[j] = message->position[index];
            if (index < message->velocity.size()) {
                velocities[j] = message->velocity[index];
            }
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_positions_ = positions;
        current_velocities_ = velocities;
        have_joint_state_ = true;
    }

    bool execute_approach() {
        if (approach_duration_ <= 0.0) {
            approach_executed_ = true;
            return true;
        }

        JointArray start;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return false;
            }
            start = current_positions_;
        }

        trajectory_msgs::msg::JointTrajectory approach;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (nominal_traj_.wp.empty()) {
                return false;
            }
            const JointArray target = to_joint_array(nominal_traj_.wp.front().q);
            const JointArray target_velocity =
                to_joint_array(nominal_traj_.wp.front().qd);
            const JointArray target_acceleration =
                to_joint_array(nominal_traj_.wp.front().qdd);
            approach = build_approach_segment(start, target, target_velocity,
                                              target_acceleration,
                                              approach_duration_, approach_dt_);
        }

        if (approach.points.empty()) {
            approach_executed_ = true;
            return true;
        }

        if (dry_run_) {
            approach_executed_ = true;
            return true;
        }

        if (!action_client_->action_server_is_ready()) return false;

        approach_executed_ = true;
        send_goal(std::move(approach), false);
        return true;
    }

    void poll_callback() {
        if (streaming_) {
            streaming_poll();
            return;
        }

        if (send_attempted_ || send_failed_hard_ || !stream_finished_.load()) return;

        if (current_goal_) return;

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (real_traj_.wp.empty()) {
                send_failed_hard_ = true;
                return;
            }
        }

        if (try_send()) send_attempted_ = true;
    }

    std::size_t chunk_samples() const {
        return std::max<std::size_t>(
            1, static_cast<std::size_t>(std::llround(chunk_duration_ / kTrajectoryDt)));
    }

    struct StreamChunk {
        trajectory_msgs::msg::JointTrajectory trajectory;
        std::size_t base{0};
        std::size_t end{0};
        double duration{0.0};
        bool first{false};
        bool covers_all{false};
    };

    void streaming_poll() {
        if (send_failed_hard_ || stream_complete_) return;

        if (approach_duration_ > 0.0 && !approach_executed_) {
            if (!execute_approach()) return;
            if (current_goal_) return;
        }

        if (current_goal_) return;

        const Clock::time_point now = Clock::now();
        if (stream_goal_sent_) {
            if (now < stream_next_send_time_) return;
            if (sending_ && !current_goal_) return;
        }
        if (stream_last_attempt_ != Clock::time_point{} &&
            now - stream_last_attempt_ < std::chrono::milliseconds(100)) {
            return;
        }
        stream_last_attempt_ = now;

        JointArray measured{};
        JointArray measured_velocity{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return;
            }
            measured = current_positions_;
            measured_velocity = current_velocities_;
        }


        const bool first = !stream_goal_sent_;
        StreamChunk chunk;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            const std::size_t count = real_traj_.wp.size();
            if (count == 0) {
                if (stream_finished_) {
                    send_failed_hard_ = true;
                }
                return;
            }

            std::size_t base = 0;
            if (!first) {
                double elapsed =
                    std::chrono::duration<double>(now - stream_send_time_).count();
                base = stream_last_base_;
                if (elapsed > 0.0) {
                    const std::size_t advance = static_cast<std::size_t>(
                        std::llround(elapsed / kTrajectoryDt));
                    const std::size_t limit =
                        std::min(stream_last_end_, count > 0 ? count - 1 : 0);
                    base = std::min(limit, base + advance);
                }
            }

            if (base >= count) {
                if (stream_finished_) {
                    stream_complete_ = true;
                }
                return;
            }

            const std::size_t end =
                stream_finished_ ? count : std::min(count, base + chunk_samples());
            const std::size_t available = end > base ? end - base : 0;
            const std::size_t minimum =
                first ? chunk_samples()
                      : std::max<std::size_t>(
                            1, static_cast<std::size_t>(std::llround(
                                   min_chunk_duration_ / kTrajectoryDt)));
            if (!stream_finished_ && available < minimum) {
                return;
            }
            if (available == 0) return;

            JointArray ramp_start = measured;
            JointArray ramp_start_velocity{};
            if (dry_run_ && !first) {
                ramp_start = to_joint_array(real_traj_.wp[base].q);
                ramp_start_velocity = to_joint_array(real_traj_.wp[base].qd);
            } else if (!first) {
                for (std::size_t j = 0; j < 7; ++j) {
                    ramp_start_velocity[j] = std::clamp(
                        measured_velocity[j], -max_ramp_velocity_, max_ramp_velocity_);
                }
            }

            try {
                if (!build_stream_chunk(ramp_start, ramp_start_velocity, first, base,
                                        end, &chunk)) {
                    return;
                }
            } catch (const std::exception&) {
                send_failed_hard_ = true;
                return;
            }
        }

        if (!dry_run_ && !action_client_->action_server_is_ready()) {
            return;
        }

        if (!dry_run_) {
            send_goal(std::move(chunk.trajectory), true);
        }

        stream_goal_sent_ = true;
        stream_send_time_ = now;
        stream_last_base_ = chunk.base;
        stream_last_end_ = chunk.end;

        const double until_next =
            std::max(chunk.duration - chunk_lead_, min_chunk_duration_);
        stream_next_send_time_ = now + std::chrono::duration_cast<Clock::duration>(
                                           std::chrono::duration<double>(until_next));

        if (stream_finished_ && chunk.covers_all) {
            stream_complete_ = true;
        }
    }

    bool build_stream_chunk(const JointArray& measured,
                            const JointArray& measured_velocity, bool first,
                            std::size_t base, std::size_t end, StreamChunk* chunk) {
        const std::size_t count = real_traj_.wp.size();
        if (end <= base || end > count) return false;

        chunk->trajectory = trajectory_msgs::msg::JointTrajectory();
        chunk->trajectory.joint_names = joint_names_;
        chunk->trajectory.header.stamp.sec = 0;
        chunk->trajectory.header.stamp.nanosec = 0;
        chunk->base = base;
        chunk->end = end;
        chunk->first = first;
        chunk->covers_all = (end == count);

        const JointArray zero{};
        const double base_time = real_traj_.wp[base].timestamp;

        if (first) {
            const JointArray target = to_joint_array(real_traj_.wp[base].q);
            const JointArray target_velocity = to_joint_array(real_traj_.wp[base].qd);
            const JointArray target_acceleration = to_joint_array(real_traj_.wp[base].qdd);

            const double start_error = max_abs(subtract(target, measured));
            if (approach_duration_ <= 0.0 && start_error > start_tolerance_) {
                throw std::runtime_error("robot too far from first trajectory point");
            }
            if (approach_duration_ > 0.0 && start_error > max_approach_distance_) {
                throw std::runtime_error("robot too far from first trajectory point");
            }

            const double offset = -base_time;
            for (std::size_t i = base; i < end; ++i) {
                const double time = offset + real_traj_.wp[i].timestamp;
                if (time <= last_point_time(chunk->trajectory) + 1.0e-9) continue;
                append_point(chunk->trajectory, time,
                             to_joint_array(real_traj_.wp[i].q),
                             to_joint_array(real_traj_.wp[i].qd),
                             to_joint_array(real_traj_.wp[i].qdd));
            }
            chunk->duration = last_point_time(chunk->trajectory);
            return true;
        }

        std::size_t ramp_samples = static_cast<std::size_t>(
            std::max(1.0, std::ceil(chunk_ramp_ / kTrajectoryDt)));
        const std::size_t span_samples = end - 1 - base;
        if (span_samples > 0) {
            ramp_samples = std::min(ramp_samples, std::max<std::size_t>(1,
                                                                        2 * span_samples / 5));
        } else {
            ramp_samples = 0;
        }
        const std::size_t target_index = base + ramp_samples;

        const JointArray target = to_joint_array(real_traj_.wp[target_index].q);
        const JointArray target_velocity =
            to_joint_array(real_traj_.wp[target_index].qd);
        const JointArray target_acceleration =
            to_joint_array(real_traj_.wp[target_index].qdd);


        





        std::cout << "\n\n\n++++++++++++++++++++\n measured = " << measured.size() << std::endl  << std::endl  << std::endl;
        // std::cout << "target = " << target << std::endl;
        // std::cout << "diff = " << current_positions_ - target << std::endl;







        const double step = max_abs(subtract(target, measured));
        if (step > max_chunk_step_) {
            return false;
        }

        double ramp_duration =
            ramp_samples > 0 ? ramp_samples * kTrajectoryDt : chunk_ramp_;
        ramp_duration = std::max(ramp_duration, 1.875 * step / max_ramp_velocity_);

        append_quintic(chunk->trajectory, 0.0, ramp_duration, approach_dt_, measured,
                       measured_velocity, zero, target, target_velocity,
                       target_acceleration);
        const double offset = ramp_duration - real_traj_.wp[target_index].timestamp;
        for (std::size_t i = target_index; i < end; ++i) {
            const double time = offset + real_traj_.wp[i].timestamp;
            if (time <= last_point_time(chunk->trajectory) + 1.0e-9) continue;
            append_point(chunk->trajectory, time,
                         to_joint_array(real_traj_.wp[i].q),
                         to_joint_array(real_traj_.wp[i].qd),
                         to_joint_array(real_traj_.wp[i].qdd));
        }
        chunk->duration = last_point_time(chunk->trajectory);
        return true;
    }

    bool try_send() {
        if (sending_ || current_goal_) {
            return false;
        }

        JointArray start{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return false;
            }
            start = current_positions_;
        }

        trajectory_msgs::msg::JointTrajectory trajectory;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (real_traj_.wp.empty()) {
                send_failed_hard_ = true;
                return false;
            }
            try {
                trajectory = build_trajectory(start);
            } catch (const std::exception&) {
                send_failed_hard_ = true;
                return false;
            }
        }

        if (dry_run_) {
            return true;
        }

        if (!action_client_->action_server_is_ready()) {
            return false;
        }

        send_goal(std::move(trajectory), false);
        return true;
    }

    void send_goal(trajectory_msgs::msg::JointTrajectory trajectory, bool quiet) {
        const uint64_t generation = ++send_generation_;
        sending_ = true;

        FollowJointTrajectory::Goal goal;
        goal.trajectory = std::move(trajectory);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.goal_response_callback =
            [this, generation, quiet](GoalHandle::SharedPtr goal_handle) {
                if (!goal_handle) {
                    if (generation == send_generation_) {
                        sending_ = false;
                        current_goal_.reset();
                    }
                    return;
                }
                if (generation != send_generation_) {
                    action_client_->async_cancel_goal(goal_handle);
                    return;
                }
                current_goal_ = goal_handle;
            };

        options.feedback_callback =
            [this](GoalHandle::SharedPtr,
                   const std::shared_ptr<const FollowJointTrajectory::Feedback>) {
            };

        options.result_callback =
            [this, generation, quiet](const GoalHandle::WrappedResult& result) {
                if (generation == send_generation_) sending_ = false;
                if (current_goal_ && result.goal_id == current_goal_->get_goal_id()) {
                    current_goal_.reset();
                }
                if (generation != send_generation_) return;
            };

        action_client_->async_send_goal(goal, options);
    }

    trajectory_msgs::msg::JointTrajectory build_approach_segment(
        const JointArray& start,
        const JointArray& target,
        const JointArray& target_velocity,
        const JointArray& target_acceleration,
        double approach_duration,
        double approach_dt) const {
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = joint_names_;
        trajectory.header.stamp.sec = 0;
        trajectory.header.stamp.nanosec = 0;

        if (!(approach_duration > 0.0)) return trajectory;

        const std::size_t approach_points = static_cast<std::size_t>(
            std::max(1.0, std::ceil(approach_duration / approach_dt)));
        trajectory.points.reserve(approach_points);

        const JointArray zero{};
        append_quintic(trajectory, 0.0, approach_duration, approach_dt,
                       start, zero, zero, target, target_velocity,
                       target_acceleration);
        return trajectory;
    }

    trajectory_msgs::msg::JointTrajectory build_trajectory(const JointArray& start) const {
        if (real_traj_.wp.empty()) throw std::runtime_error("trajectory is empty");

        const JointArray first = to_joint_array(real_traj_.wp.front().q);
        const JointArray start_delta = subtract(first, start);
        const double start_error = max_abs(start_delta);

        if (approach_duration_ <= 0.0 && start_error > start_tolerance_) {
            throw std::runtime_error("robot too far from first trajectory point");
        }
        if (approach_duration_ > 0.0 && start_error > max_approach_distance_) {
            throw std::runtime_error("robot too far from first trajectory point");
        }

        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = joint_names_;
        trajectory.header.stamp.sec = 0;
        trajectory.header.stamp.nanosec = 0;
        trajectory.points.reserve(real_traj_.wp.size());

        for (const auto& waypoint : real_traj_.wp) {
            append_point(trajectory, waypoint.timestamp,
                         to_joint_array(waypoint.q), to_joint_array(waypoint.qd),
                         to_joint_array(waypoint.qdd));
        }

        return trajectory;
    }

    void send_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (streaming_) {
            response->success = false;
            response->message =
                "streaming mode is active: the stream is followed automatically "
                "(use ~/cancel to stop)";
            return;
        }
        const bool ok = try_send();
        if (ok) {
            send_attempted_ = true;
            response->message = "trajectory sent";
        } else {
            response->message = "trajectory was not sent";
        }
        response->success = ok;
    }

    void cancel_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (streaming_) stream_complete_ = true;
        if (!current_goal_) {
            response->success = streaming_;
            response->message = streaming_ ? "streaming stopped"
                                           : "no active trajectory goal";
            return;
        }
        action_client_->async_cancel_goal(current_goal_);
        response->success = true;
        response->message = "cancel request sent";
    }

    int n_traj_;
    std::filesystem::path root_;
    std::unique_ptr<RobotModel> robot_model_;
    Trajectory nominal_traj_;
    Trajectory real_traj_;
    std::mutex trajectory_mutex_;

    std::thread receiver_thread_;
    std::atomic<bool> stop_receiver_{false};
    std::atomic<bool> stream_finished_{false};

    std::mutex state_mutex_;
    JointArray current_positions_{};
    JointArray current_velocities_{};
    bool have_joint_state_{false};

    bool send_attempted_{false};
    bool send_failed_hard_{false};
    bool sending_{false};
    uint64_t send_generation_{0};
    GoalHandle::SharedPtr current_goal_;
    bool approach_executed_{false};

    bool stream_goal_sent_{false};
    bool stream_complete_{false};
    std::size_t stream_last_base_{0};
    std::size_t stream_last_end_{0};
    Clock::time_point stream_send_time_{};
    Clock::time_point stream_next_send_time_{};
    Clock::time_point stream_last_attempt_{};

    std::string controller_action_;
    std::string joint_state_topic_;
    std::vector<std::string> joint_names_;
    bool dry_run_{false};
    double stream_timeout_{1.0};
    double approach_duration_{3.0};
    double approach_dt_{0.01};
    double max_approach_distance_{0.5};
    double start_tolerance_{0.15};
    bool streaming_{false};
    double chunk_duration_{1.0};
    double chunk_lead_{0.3};
    double chunk_ramp_{0.2};
    double min_chunk_duration_{0.3};
    double max_chunk_step_{0.35};
    double max_ramp_velocity_{1.0};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr send_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};


int main(int argc, char* argv[]) {
    int n_traj = -1;
    std::string root_arg;
    if (argc > 1) {
        try {
            std::string arg(argv[1]);
            const char* begin = arg.data();
            const char* end = arg.data() + arg.size();
            auto [ptr, ec] = std::from_chars(begin, end, n_traj);
            if (ec != std::errc{} || ptr != end || n_traj < 0) throw 1;
        } catch (int) {
            std::cerr << "Error: the trajectory index must be a valid non-negative integer."
                      << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Error: the trajectory index is required "
                     "(usage: " << argv[0] << " <traj_index> [repo_dir] [--ros-args ...])."
                  << std::endl;
        return 1;
    }
    if (argc > 2 && std::string(argv[2]).rfind("--", 0) != 0) root_arg = argv[2];

    rclcpp::init(argc, argv);
    const std::filesystem::path root = findProjectRoot(root_arg);

    int status = 0;
    try {
        auto node = std::make_shared<RobotControllerNode>(n_traj, root);
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
