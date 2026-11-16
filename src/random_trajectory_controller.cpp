/*
random_trajectory_controller.cpp - Send one point-to-point trajectory to a real Panda.

This is intentionally a small, single-purpose node:

  1. read the waypoints of a trajectory set (default:
     /home/lab/SkeletonTracking/src/trajectories/test3, i.e. p_ref.csv +
     t_ref.csv; q_ref.csv is accepted as well);
  2. convert the Cartesian waypoints to joint configurations with the Panda
     inverse kinematics (the project's RobotModel), or use the joint waypoints
     directly;
  3. join the waypoints with quintic point-to-point segments and sample the
     result at 60 Hz (the project's build_p2p_traj);
  4. prepend a smooth move from the measured joint state to the first sample;
  5. send the whole thing as a single
     control_msgs::action::FollowJointTrajectory goal to the
     joint_trajectory_controller started by multipanda_ros2.

The node is a pure action client.  The robot hardware and the controller are
started by launch/random_trajectory.launch.py (or by any other multipanda_ros2
bringup); this file does not use libfranka directly.

Run with the multipanda stack already up:

    ./build_ros2/random_trajectory_controller --ros-args \
        -p trajectory_dir:=/home/lab/SkeletonTracking/src/trajectories/test3

Useful parameters:
    trajectory_dir:=<path>          trajectory set directory (default test3)
    robot_urdf:=<path>              Panda URDF (default: <repo>/src/urdf/panda.urdf)
    controller_action:=<name>       default /panda_effort_jtc/follow_joint_trajectory
    joint_state_topic:=<name>       default /joint_states
    sample_rate:=60.0              trajectory sampling rate (Hz)
    approach_duration:=3.0         smooth move to the first point (s)
    max_approach_distance:=2.0     refuse a larger initial offset (rad)
    start_delay:=1.0               wait after the first joint state (s)
    dry_run:=false                 build and log the goal without sending it

Safety: keep the E-stop within reach and the workspace clear before launching
with a trajectory that moves the arm.  The node refuses to start when the robot
is farther than max_approach_distance from the first trajectory point.
*/

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "project_root.hpp"
#include "robot_model.hpp"
#include "trajectory_loader.hpp"
#include "trajectory_utils.hpp"

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

constexpr std::size_t kNumJoints = 7;
using JointArray = std::array<double, kNumJoints>;

std::vector<std::string> default_joint_names() {
    return {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
            "panda_joint5", "panda_joint6", "panda_joint7"};
}

JointArray to_joint_array(const Eigen::VectorXd& q) {
    if (q.size() != static_cast<Eigen::Index>(kNumJoints)) {
        throw std::runtime_error("expected a 7-element joint vector, got " +
                                 std::to_string(q.size()));
    }
    JointArray result{};
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        result[j] = q[static_cast<Eigen::Index>(j)];
    }
    return result;
}

double max_abs_difference(const JointArray& a, const JointArray& b) {
    double value = 0.0;
    for (std::size_t j = 0; j < kNumJoints; ++j) {
        value = std::max(value, std::abs(a[j] - b[j]));
    }
    return value;
}

void set_point_duration(trajectory_msgs::msg::JointTrajectoryPoint& point,
                        double seconds) {
    const int64_t nanoseconds = static_cast<int64_t>(std::llround(seconds * 1.0e9));
    point.time_from_start.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    point.time_from_start.nanosec =
        static_cast<uint32_t>(nanoseconds % 1000000000LL);
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

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// One-shot FollowJointTrajectory client.
// ─────────────────────────────────────────────────────────────────────────────
class RandomTrajectoryController : public rclcpp::Node {
public:
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    RandomTrajectoryController() : Node("random_trajectory_controller") {
        declare_parameters();
        validate_parameters();

        if (robot_urdf_.empty()) {
            robot_urdf_ = (findProjectRoot() / "src" / "urdf" / "panda.urdf").string();
        }

        robot_model_ = std::make_unique<RobotModel>(robot_urdf_);
        load_and_sample_trajectory();

        joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, rclcpp::QoS(10),
            std::bind(&RandomTrajectoryController::joint_state_callback, this,
                      std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, controller_action_);

        start_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RandomTrajectoryController::start_timer_callback, this));

        RCLCPP_INFO(get_logger(), "Random trajectory controller ready.");
        RCLCPP_INFO(get_logger(), "  trajectory dir  : %s", trajectory_dir_.c_str());
        RCLCPP_INFO(get_logger(), "  URDF            : %s", robot_urdf_.c_str());
        RCLCPP_INFO(get_logger(), "  action server   : %s", controller_action_.c_str());
        RCLCPP_INFO(get_logger(), "  joint states    : %s", joint_state_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  sampling rate   : %.1f Hz", sample_rate_);
        RCLCPP_INFO(get_logger(), "  dry run         : %s", dry_run_ ? "true" : "false");
        RCLCPP_INFO(get_logger(),
                    "Waiting for joint states and the action server, then sending the "
                    "trajectory.");
    }

private:
    // ── Parameters ────────────────────────────────────────────────────────────
    void declare_parameters() {
        trajectory_dir_ = declare_parameter<std::string>(
            "trajectory_dir", "/home/lab/SkeletonTracking/src/trajectories/test3");
        robot_urdf_ = declare_parameter<std::string>("robot_urdf", "");
        controller_action_ = declare_parameter<std::string>(
            "controller_action", "/panda_effort_jtc/follow_joint_trajectory");
        joint_state_topic_ = declare_parameter<std::string>(
            "joint_state_topic", "/joint_states");
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", default_joint_names());
        sample_rate_ = declare_parameter<double>("sample_rate", 60.0);
        approach_duration_ = declare_parameter<double>("approach_duration", 3.0);
        max_approach_distance_ =
            declare_parameter<double>("max_approach_distance", 2.0);
        start_tolerance_ = declare_parameter<double>("start_tolerance", 0.15);
        start_delay_ = declare_parameter<double>("start_delay", 1.0);
        dry_run_ = declare_parameter<bool>("dry_run", false);
    }

    void validate_parameters() const {
        if (joint_names_.size() != kNumJoints) {
            throw std::runtime_error("joint_names must contain exactly 7 names");
        }
        if (!(sample_rate_ > 0.0)) {
            throw std::runtime_error("sample_rate must be greater than zero");
        }
        if (approach_duration_ < 0.0) {
            throw std::runtime_error("approach_duration must be zero or positive");
        }
        if (max_approach_distance_ < 0.0) {
            throw std::runtime_error(
                "max_approach_distance must be zero or positive");
        }
        if (start_tolerance_ < 0.0) {
            throw std::runtime_error("start_tolerance must be zero or positive");
        }
        if (start_delay_ < 0.0) {
            throw std::runtime_error("start_delay must be zero or positive");
        }
    }

    // ── Trajectory loading ────────────────────────────────────────────────────
    // Convert the loaded waypoints to joint space.  Cartesian waypoints need
    // the inverse kinematics, exactly like trajectory_utils::load_trajectory.
    std::vector<Eigen::VectorXd> joint_waypoints(const TrajectoryData& data) const {
        if (data.isJointSpace()) {
            return data.q;
        }

        Eigen::VectorXd q_base(kNumJoints);
        q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

        // Only the position is taken from p_ref.csv; the end effector keeps
        // this fixed orientation (the same convention as the project loader).
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        rotation(1, 1) = -1.0;
        rotation(2, 2) = -1.0;

        std::vector<Eigen::VectorXd> result;
        result.reserve(data.p.size());

        for (std::size_t i = 0; i < data.p.size(); ++i) {
            Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
            target.linear() = rotation;
            target.translation() = data.p[i];

            Eigen::VectorXd q;
            bool ok = robot_model_->ComputeIK("panda_link8", target, q_base, &q);
            if (!ok && !result.empty()) {
                // The previous solution is usually closer to the new point.
                ok = robot_model_->ComputeIK("panda_link8", target, result.back(), &q);
            }
            if (!ok) {
                throw std::runtime_error("IK failed for waypoint " +
                                         std::to_string(i) + " of " +
                                         trajectory_dir_);
            }
            result.push_back(std::move(q));
        }
        return result;
    }

    // Load test3, solve the IK if needed and sample every quintic segment at
    // sample_rate_ (60 Hz by default).
    void load_and_sample_trajectory() {
        const std::filesystem::path directory(trajectory_dir_);
        if (!std::filesystem::is_directory(directory)) {
            throw std::runtime_error("trajectory_dir is not a directory: " +
                                     trajectory_dir_);
        }

        const TrajectoryData data = loadTrajectorySet(directory);
        if (data.t.size() < 2) {
            throw std::runtime_error("the trajectory needs at least 2 waypoints");
        }

        const std::vector<Eigen::VectorXd> waypoints = joint_waypoints(data);
        if (waypoints.size() != data.t.size()) {
            throw std::runtime_error("waypoint and time count do not match");
        }

        const double dt = 1.0 / sample_rate_;
        sampled_trajectory_ = Trajectory{};
        sampled_trajectory_.collision = false;

        double sample_index = 0.0;
        for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
            const double duration = data.t[i + 1] - data.t[i];
            if (duration <= 0.0) {
                continue;  // zero-length segment: nothing to sample
            }
            build_p2p_traj(waypoints[i], waypoints[i + 1], nullptr,
                           sampled_trajectory_, duration, sample_index,
                           *robot_model_, dt);
        }

        if (sampled_trajectory_.wp.empty()) {
            throw std::runtime_error("the sampled trajectory is empty");
        }

        // build_p2p_traj samples [0, duration) for every segment, so the last
        // waypoint of the file is not included.  Append it explicitly with zero
        // velocity/acceleration so the goal ends exactly on the final point.
        Waypoint endpoint;
        endpoint.q = waypoints.back();
        endpoint.qd = Eigen::VectorXd::Zero(kNumJoints);
        endpoint.qdd = Eigen::VectorXd::Zero(kNumJoints);
        endpoint.timestamp = sampled_trajectory_.wp.back().timestamp + dt;
        sampled_trajectory_.wp.push_back(std::move(endpoint));

        RCLCPP_INFO(get_logger(),
                    "Loaded %zu waypoints from '%s' and sampled %zu points at %.1f Hz "
                    "(%.3f s).",
                    waypoints.size(), trajectory_dir_.c_str(),
                    sampled_trajectory_.wp.size(), sample_rate_,
                    sampled_trajectory_.wp.back().timestamp);
    }

    // ── Goal construction ─────────────────────────────────────────────────────
    trajectory_msgs::msg::JointTrajectory build_goal(const JointArray& start) const {
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = joint_names_;
        // A zero stamp tells JointTrajectoryController to start on the first
        // sample as soon as the goal is accepted.
        trajectory.header.stamp.sec = 0;
        trajectory.header.stamp.nanosec = 0;

        const Waypoint& first = sampled_trajectory_.wp.front();
        const JointArray first_joint = to_joint_array(first.q);
        const double start_error = max_abs_difference(first_joint, start);

        if (approach_duration_ <= 0.0 && start_error > start_tolerance_) {
            throw std::runtime_error(
                "the robot is " + std::to_string(start_error) +
                " rad from the first trajectory point (start_tolerance = " +
                std::to_string(start_tolerance_) + " rad)");
        }
        if (approach_duration_ > 0.0 && start_error > max_approach_distance_) {
            throw std::runtime_error(
                "the robot is " + std::to_string(start_error) +
                " rad from the first trajectory point, which exceeds "
                "max_approach_distance = " +
                std::to_string(max_approach_distance_) + " rad");
        }

        const double dt = 1.0 / sample_rate_;
        double offset = 0.0;

        // Smooth quintic approach from the measured pose to the first sample.
        if (approach_duration_ > 0.0) {
            Eigen::VectorXd start_eigen(kNumJoints);
            for (std::size_t j = 0; j < kNumJoints; ++j) {
                start_eigen[static_cast<Eigen::Index>(j)] = start[j];
            }

            Trajectory approach;
            approach.collision = false;
            double approach_index = 0.0;
            build_p2p_traj(start_eigen, first.q, nullptr, approach,
                           approach_duration_, approach_index, *robot_model_, dt);

            for (const Waypoint& waypoint : approach.wp) {
                append_waypoint(trajectory, waypoint.timestamp, waypoint);
            }
            offset = approach_index * dt;
        }

        // The sampled trajectory itself, shifted after the approach.
        for (const Waypoint& waypoint : sampled_trajectory_.wp) {
            append_waypoint(trajectory, offset + waypoint.timestamp, waypoint);
        }

        return trajectory;
    }

    void log_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) const {
        double max_velocity = 0.0;
        double max_acceleration = 0.0;
        for (const auto& point : trajectory.points) {
            for (double value : point.velocities) {
                max_velocity = std::max(max_velocity, std::abs(value));
            }
            for (double value : point.accelerations) {
                max_acceleration = std::max(max_acceleration, std::abs(value));
            }
        }

        const auto& last = trajectory.points.back().time_from_start;
        const double duration = static_cast<double>(last.sec) + 1.0e-9 * last.nanosec;
        RCLCPP_INFO(
            get_logger(),
            "Goal: %zu points, %.3f s, max |qd| %.3f rad/s, max |qdd| %.3f rad/s^2.",
            trajectory.points.size(), duration, max_velocity, max_acceleration);
    }

    // ── ROS callbacks ─────────────────────────────────────────────────────────
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message) {
        JointArray positions{};

        for (std::size_t j = 0; j < kNumJoints; ++j) {
            const auto iterator = std::find(message->name.begin(), message->name.end(),
                                            joint_names_[j]);
            if (iterator == message->name.end()) {
                return;  // incomplete message
            }
            const auto index = static_cast<std::size_t>(
                std::distance(message->name.begin(), iterator));
            if (index >= message->position.size()) {
                return;
            }
            positions[j] = message->position[index];
        }

        std::lock_guard<std::mutex> lock(state_mutex_);
        current_positions_ = positions;
        have_joint_state_ = true;
        if (!have_first_joint_state_) {
            have_first_joint_state_ = true;
            first_joint_state_time_ = now();
            RCLCPP_INFO(get_logger(), "Received the first complete joint state.");
        }
    }

    void start_timer_callback() {
        if (start_attempted_) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return;
            }
        }

        if ((now() - first_joint_state_time_).seconds() < start_delay_) {
            return;
        }

        if (!dry_run_ && !action_client_->action_server_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for action server '%s'.",
                                 controller_action_.c_str());
            return;
        }

        start_attempted_ = true;
        start_timer_->cancel();
        send_trajectory();
    }

    void send_trajectory() {
        JointArray start{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return;
            }
            start = current_positions_;
        }

        trajectory_msgs::msg::JointTrajectory trajectory;
        try {
            trajectory = build_goal(start);
        } catch (const std::exception& exception) {
            RCLCPP_ERROR(get_logger(), "Cannot build the target trajectory: %s",
                         exception.what());
            return;
        }

        log_trajectory(trajectory);

        if (dry_run_) {
            RCLCPP_INFO(get_logger(),
                        "dry_run is set: the trajectory was built but not sent.");
            return;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory = std::move(trajectory);

        auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        options.goal_response_callback =
            [this](GoalHandle::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(),
                                 "The trajectory goal was rejected by the controller.");
                    return;
                }
                current_goal_ = goal_handle;
                RCLCPP_INFO(get_logger(),
                            "Trajectory goal accepted; the robot is executing it.");
            };

        options.feedback_callback =
            [this](GoalHandle::SharedPtr,
                   const std::shared_ptr<const FollowJointTrajectory::Feedback>
                       feedback) {
                const auto current_time = now();
                if ((current_time - last_feedback_time_).seconds() < 1.0) {
                    return;
                }
                last_feedback_time_ = current_time;

                double max_error = 0.0;
                for (double value : feedback->error.positions) {
                    max_error = std::max(max_error, std::abs(value));
                }
                RCLCPP_INFO(get_logger(),
                            "Tracking max |position error| = %.4f rad.", max_error);
            };

        options.result_callback =
            [this](const GoalHandle::WrappedResult& result) {
                current_goal_.reset();
                const auto code = result.result ? result.result->error_code : -999;
                const auto message =
                    result.result ? result.result->error_string : "no result";
                RCLCPP_INFO(get_logger(), "Trajectory finished with code %s (%d): %s",
                            result_code_to_string(code).c_str(),
                            static_cast<int>(code), message.c_str());
            };

        action_client_->async_send_goal(goal, options);
        RCLCPP_INFO(get_logger(), "Sent trajectory goal to '%s'.",
                    controller_action_.c_str());
    }

    // ── Parameters ────────────────────────────────────────────────────────────
    std::string trajectory_dir_;
    std::string robot_urdf_;
    std::string controller_action_;
    std::string joint_state_topic_;
    std::vector<std::string> joint_names_;
    double sample_rate_{60.0};
    double approach_duration_{3.0};
    double max_approach_distance_{2.0};
    double start_tolerance_{0.15};
    double start_delay_{1.0};
    bool dry_run_{false};

    // ── Trajectory data ───────────────────────────────────────────────────────
    std::unique_ptr<RobotModel> robot_model_;
    Trajectory sampled_trajectory_;

    // ── State ─────────────────────────────────────────────────────────────────
    std::mutex state_mutex_;
    JointArray current_positions_{};
    bool have_joint_state_{false};
    bool have_first_joint_state_{false};
    rclcpp::Time first_joint_state_time_;
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
    bool start_attempted_{false};
    GoalHandle::SharedPtr current_goal_;

    // ── ROS interfaces ────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscription_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr start_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RandomTrajectoryController>();
        rclcpp::spin(node);
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("random_trajectory_controller"), "%s",
                     exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
