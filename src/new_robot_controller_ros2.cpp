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
#include "data_transmitter.hpp"
#include "utils.hpp"


#include "controller_utils.hpp"


// ─────────────────────────────────────────────────────────────────────────────
// One-shot FollowJointTrajectory client.
// ─────────────────────────────────────────────────────────────────────────────
class RobotControllerNode : public rclcpp::Node {
public:
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    RobotControllerNode() : Node("random_trajectory_controller") {
        declare_parameters();
        validate_parameters();

        if (robot_urdf_.empty()) {
            robot_urdf_ = (findProjectRoot() / "src" / "urdf" / "panda.urdf").string();
        }

        robot_model_ = std::make_unique<RobotModel>(robot_urdf_);
        load_and_sample_trajectory();

        // Locate the project root so the nominal trajectory can be found
        // relative to src/trajectories/test<n_traj_>.
        root_ = findProjectRoot();

        std::cout << "\n++++++++++ !loading traj \n" << std::endl;
        nominal_traj_ = load_trajectory(*robot_model_, root_.string() + "/", n_traj_, kStreamDt);
        std::cout << "\n++++++++++ !traj loaded \n" << std::endl;
        if (nominal_traj_.wp.empty()) {
            throw std::runtime_error("the nominal trajectory is empty");
        }

        joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_state_topic_, rclcpp::QoS(10),
            std::bind(&RobotControllerNode::joint_state_callback, this,
                      std::placeholders::_1));

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, controller_action_);

        receiver_thread_ = std::thread(&RobotControllerNode::receiver_loop, this);

        controller_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RobotControllerNode::controller_callback, this));

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



    void receiver_loop() {
        // Instead of reading incoming waypoints from ZMQ, stream the nominal
        // trajectory into real_traj_ one segment at a time.  Each consecutive
        // pair of nominal waypoints is joined with a quintic segment sampled
        // at kTrajectoryDt, accumulating the timestamp in `t`.
        double t = 0.0;
        std::vector<double> vnorm;

        for (std::size_t i = 0; i + 1 < nominal_traj_.wp.size(); ++i) {
            if (stop_receiver_.load() || !rclcpp::ok()) {
                break;
            }

            const Waypoint& previous = nominal_traj_.wp[i];
            const Waypoint& next = nominal_traj_.wp[i + 1];

            {
                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                build_p2p_traj(previous.q, next.q, &vnorm, real_traj_, kStreamDt, t,
                               *robot_model_, kTrajectoryDt, previous.qd,
                               previous.qdd, next.qd, next.qdd);
            }
        }

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            stream_finished_.store(true);
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


    bool execute_approach() {
        if (approach_duration_ <= 0.0) {
            approach_executed_ = true;
            return true;
        }

        JointArray start;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                std::cout << "\n++++++++++ !have_joint_state_ \n" << std::endl;
                return false;
            }
            start = current_positions_;
        }

        trajectory_msgs::msg::JointTrajectory approach;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (nominal_traj_.wp.empty()) {

                std::cout << "\n++++++++++ nominal_traj_.wp.empty() \n" << std::endl;
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

    void controller_callback() {


        std::cout << "\n++++++++++ looping \n" << std::endl;



        if (start_attempted_) {
            std::cout << "\n++++++++++ start_attempted_ \n" << std::endl;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                std::cout << "\n++++++++++ have_joint_state_ \n" << std::endl;
                return;
            }
        }

        if ((now() - first_joint_state_time_).seconds() < start_delay_) {
            std::cout << "\n++++++++++ first_joint_state_time_ \n" << std::endl;
            return;
        }

        if (!dry_run_ && !action_client_->action_server_is_ready()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Waiting for action server '%s'.",
                                 controller_action_.c_str());
            return;
        }

        // std::cout << "LOOPING" << std::endl;

        if (approach_duration_ > 0.0 && !approach_executed_) {
            std::cout << "\n++++++++++ approach_\n" << std::endl;
            if (!execute_approach()) return;
            // The approach goal was sent asynchronously: current_goal_ is not
            // populated yet (the goal_response callback runs on the spin
            // thread).  Wait for the approach to complete before sending the
            // main trajectory so the two goals do not collide.
            return;
        }

        // Wait for an in-flight approach (or any other goal) to finish before
        // sending the main trajectory.
        if (current_goal_) return;

        start_attempted_ = true;
        // controller_->cancel();
        send_trajectory();
    }







    void send_trajectory() {

        std::cout << "\n++++++++++ send_trajectory \n" << std::endl;

        JointArray start{};
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!have_joint_state_) {
                return;
            }
            start = current_positions_;
        }

        // trajectory_msgs::msg::JointTrajectory trajectory;
        // try {
        //     trajectory = build_goal(start);
        // } catch (const std::exception& exception) {
        //     RCLCPP_ERROR(get_logger(), "Cannot build the target trajectory: %s",
        //                  exception.what());
        //     return;
        // }

        trajectory_msgs::msg::JointTrajectory trajectory;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (real_traj_.wp.empty()) {
                std::cout << "\n++++++++++ real_traj_.wp.empty() \n" << std::endl;
                send_failed_hard_ = true;
                return;
            }
            try {
                std::cout << "\n++++++++++ try \n" << std::endl;
                trajectory = build_trajectory(start);
            } catch (const std::exception&) {
                send_failed_hard_ = true;
                return;
            }
        }

        log_trajectory(trajectory);

        if (dry_run_) {
            RCLCPP_INFO(get_logger(),
                        "dry_run is set: the trajectory was built but not sent.");
            return;
        }

        
        send_goal(std::move(trajectory), false);
    }



    void send_goal(trajectory_msgs::msg::JointTrajectory trajectory, bool quiet) {

        std::cout << "\n++++++++++ send_goal \n" << std::endl;


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











    int n_traj_ = 3;
    std::filesystem::path root_;
    Trajectory nominal_traj_;
    Trajectory real_traj_;
    std::mutex trajectory_mutex_;

    std::thread receiver_thread_;
    std::atomic<bool> stop_receiver_{false};
    std::atomic<bool> stream_finished_{false};

    
    bool send_attempted_{false};
    bool send_failed_hard_{false};
    bool sending_{false};
    uint64_t send_generation_{0};
    bool approach_executed_{false};

    bool stream_goal_sent_{false};
    bool stream_complete_{false};
    std::size_t stream_last_base_{0};
    std::size_t stream_last_end_{0};
    Clock::time_point stream_send_time_{};
    Clock::time_point stream_next_send_time_{};
    Clock::time_point stream_last_attempt_{};

    double stream_timeout_{1.0};
    double approach_dt_{0.01};
    bool streaming_{false};
    double chunk_duration_{1.0};
    double chunk_lead_{0.3};
    double chunk_ramp_{0.2};
    double min_chunk_duration_{0.3};
    double max_chunk_step_{0.35};
    double max_ramp_velocity_{1.0};

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
    JointArray current_velocities_{};
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
    rclcpp::TimerBase::SharedPtr controller_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<RobotControllerNode>();
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
