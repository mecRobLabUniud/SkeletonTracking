#ifndef CONTROLLER_UTILS_HPP
#define CONTROLLER_UTILS_HPP

// Declarations for the free functions and constants that were moved out of
// new_robot_controller_ros2.cpp (formerly an anonymous namespace) into
// controller_utils.cpp.  new_robot_controller_ros2.cpp includes this header so
// it can use these symbols; controller_utils.cpp defines them.

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"

// ── Constants ────────────────────────────────────────────────────────────────
constexpr std::size_t kNumJoints = 7;
constexpr double kRateHz = 60.0;
constexpr double kStreamDt = 1.0 / kRateHz;
constexpr double kTrajectoryDt = 0.001;
constexpr int kRobotZmqDevice = 12;
constexpr int kZmqBasePort = 6000;

// ── Type aliases ─────────────────────────────────────────────────────────────
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using JointArray = std::array<double, 7>;
using Clock = std::chrono::steady_clock;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// ── Free functions ───────────────────────────────────────────────────────────
std::vector<std::string> default_joint_names();

JointArray to_joint_array(const Eigen::VectorXd& value);

bool all_finite(const JointArray& q);

double max_abs(const JointArray& q);

JointArray subtract(const JointArray& a, const JointArray& b);

JointArray add_scaled(const JointArray& base, const JointArray& delta,
                      double scale);

JointArray scaled(const JointArray& value, double scale);

void set_point_duration(trajectory_msgs::msg::JointTrajectoryPoint& point,
                        double seconds);

void append_point(trajectory_msgs::msg::JointTrajectory& trajectory,
                  double time_from_start, const JointArray& positions,
                  const JointArray& velocities,
                  const JointArray& accelerations);

double last_point_time(const trajectory_msgs::msg::JointTrajectory& trajectory);

double max_abs_difference(const JointArray& a, const JointArray& b);

void append_waypoint(trajectory_msgs::msg::JointTrajectory& trajectory,
                     double time_from_start, const Waypoint& waypoint);

void append_quintic(trajectory_msgs::msg::JointTrajectory& trajectory,
                    double t0, double duration, double dt,
                    const JointArray& p0, const JointArray& v0,
                    const JointArray& a0, const JointArray& p1,
                    const JointArray& v1, const JointArray& a1);

Waypoint receiveRobotStates(DataTransmitter& transmitter);

std::string result_code_to_string(int32_t code);

#endif  // CONTROLLER_UTILS_HPP
