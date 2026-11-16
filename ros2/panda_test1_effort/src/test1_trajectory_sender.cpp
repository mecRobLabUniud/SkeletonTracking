// Copyright 2026 SkeletonTracking
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * test1_trajectory_sender
 * -----------------------
 * Reads the dense joint trajectory produced by SkeletonTracking
 * (q.csv / qd.csv / qdd.csv, 1 kHz) and sends it to a Franka Emika Panda
 * through a control_msgs/FollowJointTrajectory action.
 *
 * The receiving controller must be a joint_trajectory_controller whose
 * command interface is "effort" (see config/panda_effort_jtc.yaml).  In that
 * configuration ros2_control's JointTrajectoryController runs a PID per joint
 * and writes the resulting torque to the Franka hardware interface, which
 * libfranka executes in torque mode.  Gravity is compensated by the robot
 * itself; the trajectory velocities are not used as a torque feed-forward
 * (ff_velocity_scale is set to zero in the supplied configuration).
 *
 * The node prepends a smooth quintic move from the robot's current joint
 * position to the first trajectory sample, so the effort controller does not
 * see a position step when the goal is accepted.  The approach is refused if
 * the robot is farther than max_approach_distance from the first sample.  The
 * approach can be disabled (approach_duration:=0); in that case the robot must
 * already be at the start pose (checked against start_tolerance).
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
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

namespace
{

constexpr std::size_t kNumJoints = 7;
using JointArray = std::array<double, kNumJoints>;

std::vector<std::string> default_joint_names()
{
  return {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
          "panda_joint5", "panda_joint6", "panda_joint7"};
}

bool all_finite(const JointArray & q)
{
  return std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); });
}

double max_abs(const JointArray & q)
{
  double value = 0.0;
  for (double v : q) {
    value = std::max(value, std::abs(v));
  }
  return value;
}

JointArray subtract(const JointArray & a, const JointArray & b)
{
  JointArray result{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    result[i] = a[i] - b[i];
  }
  return result;
}

JointArray add_scaled(const JointArray & base, const JointArray & delta, double scale)
{
  JointArray result{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    result[i] = base[i] + scale * delta[i];
  }
  return result;
}

JointArray scaled(const JointArray & value, double scale)
{
  JointArray result{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    result[i] = value[i] * scale;
  }
  return result;
}

// First derivative of a uniform time series (central differences in the
// interior, one-sided differences at the ends).  Used when qd.csv / qdd.csv
// are not present, so that q.csv alone is enough to execute the trajectory.
std::vector<JointArray> finite_difference(
  const std::vector<JointArray> & values, double dt)
{
  const std::size_t n = values.size();
  std::vector<JointArray> derivative(n, JointArray{});
  if (n < 2 || !(dt > 0.0)) {
    return derivative;
  }

  derivative.front() = scaled(subtract(values[1], values[0]), 1.0 / dt);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    derivative[i] =
      scaled(subtract(values[i + 1], values[i - 1]), 1.0 / (2.0 * dt));
  }
  derivative.back() =
    scaled(subtract(values[n - 1], values[n - 2]), 1.0 / dt);

  return derivative;
}

// Panda joint position limits (rad), used only for sanity warnings.
constexpr std::array<double, kNumJoints> kLowerLimits = {
  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, kNumJoints> kUpperLimits = {
  2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};

void set_point_duration(
  trajectory_msgs::msg::JointTrajectoryPoint & point, double seconds)
{
  const int64_t nanoseconds = static_cast<int64_t>(std::llround(seconds * 1.0e9));
  point.time_from_start.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  point.time_from_start.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
}

std::string result_code_to_string(int32_t code)
{
  switch (code) {
    case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
      return "SUCCESSFUL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
      return "INVALID_GOAL";
    case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "UNKNOWN(" + std::to_string(code) + ")";
  }
}

}  // namespace

class Test1TrajectorySender : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  Test1TrajectorySender()
  : Node("test1_trajectory_sender")
  {
    declare_parameters();
    validate_parameters();

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::QoS(10),
      std::bind(&Test1TrajectorySender::joint_state_callback, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, controller_action_);

    start_service_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      std::bind(
        &Test1TrajectorySender::start_service_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel",
      std::bind(
        &Test1TrajectorySender::cancel_service_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    try {
      load_trajectory();
    } catch (const std::exception & exception) {
      load_error_ = exception.what();
      RCLCPP_ERROR(
        get_logger(), "Could not load trajectory from '%s': %s", trajectory_dir_.c_str(),
        load_error_.c_str());
    }

    if (auto_start_) {
      readiness_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Test1TrajectorySender::readiness_timer_callback, this));
    }

    RCLCPP_INFO(get_logger(), "Trajectory sender ready.");
    RCLCPP_INFO(get_logger(), "  controller action : %s", controller_action_.c_str());
    RCLCPP_INFO(get_logger(), "  joint states      : %s", joint_state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  trajectory dir    : %s", trajectory_dir_.c_str());
    RCLCPP_INFO(get_logger(), "  trajectory file   : %s", trajectory_file_.c_str());
    RCLCPP_INFO(get_logger(), "  auto_start        : %s", auto_start_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  dry_run           : %s", dry_run_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(), "  Trigger a run with: ros2 service call %s/start std_srvs/srv/Trigger {}",
      get_fully_qualified_name());
  }

private:
  void declare_parameters()
  {
    trajectory_dir_ = declare_parameter<std::string>(
      "trajectory_dir", "/home/lab/SkeletonTracking/src/trajectories/test1");
    trajectory_file_ = declare_parameter<std::string>("trajectory_file", "q.csv");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", default_joint_names());
    sample_period_ = declare_parameter<double>("sample_period", 0.001);
    time_scale_ = declare_parameter<double>("time_scale", 1.0);
    approach_duration_ = declare_parameter<double>("approach_duration", 3.0);
    approach_dt_ = declare_parameter<double>("approach_dt", 0.01);
    max_approach_distance_ = declare_parameter<double>("max_approach_distance", 0.5);
    start_tolerance_ = declare_parameter<double>("start_tolerance", 0.15);
    start_delay_ = declare_parameter<double>("start_delay", 1.0);
    auto_start_ = declare_parameter<bool>("auto_start", true);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    controller_action_ = declare_parameter<std::string>(
      "controller_action", "/panda_effort_jtc/follow_joint_trajectory");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
  }

  void validate_parameters()
  {
    if (joint_names_.size() != kNumJoints) {
      throw std::runtime_error(
        "joint_names must contain exactly " + std::to_string(kNumJoints) +
        " names, got " + std::to_string(joint_names_.size()));
    }
    if (!(sample_period_ > 0.0)) {
      throw std::runtime_error("sample_period must be greater than zero");
    }
    if (trajectory_file_.empty()) {
      throw std::runtime_error("trajectory_file must not be empty");
    }
    if (!(time_scale_ > 0.0)) {
      throw std::runtime_error("time_scale must be greater than zero");
    }
    if (approach_duration_ < 0.0) {
      throw std::runtime_error("approach_duration must be zero or positive");
    }
    if (max_approach_distance_ < 0.0) {
      throw std::runtime_error("max_approach_distance must be zero or positive");
    }
    if (!(approach_dt_ > 0.0)) {
      throw std::runtime_error("approach_dt must be greater than zero");
    }
    if (start_delay_ < 0.0) {
      throw std::runtime_error("start_delay must be zero or positive");
    }
    if (start_tolerance_ < 0.0) {
      throw std::runtime_error("start_tolerance must be zero or positive");
    }
  }

  // ── Trajectory loading ────────────────────────────────────────────────────

  std::vector<JointArray> load_joint_csv(const std::filesystem::path & path) const
  {
    std::ifstream file(path);
    if (!file) {
      throw std::runtime_error("cannot open '" + path.string() + "'");
    }

    std::vector<JointArray> rows;
    std::string line;
    std::size_t line_number = 0;

    while (std::getline(file, line)) {
      ++line_number;
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      const auto first = line.find_first_not_of(" \t");
      if (first == std::string::npos || line[first] == '#') {
        continue;
      }

      // Accept both comma separated and whitespace separated values.
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream stream(line);
      JointArray row{};
      std::size_t column = 0;
      double value = 0.0;
      while (stream >> value) {
        if (column >= kNumJoints) {
          throw std::runtime_error(
            "'" + path.string() + "' line " + std::to_string(line_number) +
            " has more than " + std::to_string(kNumJoints) + " columns");
        }
        row[column++] = value;
      }

      if (column == 0) {
        continue;
      }
      if (column != kNumJoints) {
        throw std::runtime_error(
          "'" + path.string() + "' line " + std::to_string(line_number) +
          " has " + std::to_string(column) + " columns, expected " +
          std::to_string(kNumJoints));
      }
      if (!all_finite(row)) {
        throw std::runtime_error(
          "'" + path.string() + "' line " + std::to_string(line_number) +
          " contains a non-finite value");
      }
      rows.push_back(row);
    }

    if (rows.empty()) {
      throw std::runtime_error("'" + path.string() + "' contains no samples");
    }
    return rows;
  }

  void load_trajectory()
  {
    const std::filesystem::path directory(trajectory_dir_);
    if (!std::filesystem::is_directory(directory)) {
      throw std::runtime_error("trajectory_dir is not a directory: " + trajectory_dir_);
    }

    std::filesystem::path q_path(trajectory_file_);
    if (q_path.is_relative()) {
      q_path = directory / q_path;
    }
    if (!std::filesystem::exists(q_path)) {
      throw std::runtime_error("trajectory file does not exist: " + q_path.string());
    }

    q_ = load_joint_csv(q_path);
    RCLCPP_INFO(get_logger(), "Read joint positions from '%s'.", q_path.c_str());

    const auto qd_path = directory / "qd.csv";
    if (std::filesystem::exists(qd_path)) {
      qd_ = load_joint_csv(qd_path);
    } else {
      qd_ = finite_difference(q_, sample_period_);
      RCLCPP_INFO(
        get_logger(), "qd.csv not found, derived joint velocities from '%s'.",
        q_path.filename().c_str());
    }

    const auto qdd_path = directory / "qdd.csv";
    if (std::filesystem::exists(qdd_path)) {
      qdd_ = load_joint_csv(qdd_path);
    } else {
      qdd_ = finite_difference(qd_, sample_period_);
      RCLCPP_INFO(get_logger(), "qdd.csv not found, derived joint accelerations.");
    }

    if (qd_.size() != q_.size() || qdd_.size() != q_.size()) {
      throw std::runtime_error(
        "'" + q_path.string() + "', qd.csv and qdd.csv have different numbers of samples: " +
        std::to_string(q_.size()) + ", " + std::to_string(qd_.size()) + ", " +
        std::to_string(qdd_.size()));
    }

    // Sanity check: position limits.
    for (std::size_t i = 0; i < q_.size(); ++i) {
      for (std::size_t j = 0; j < kNumJoints; ++j) {
        if (q_[i][j] < kLowerLimits[j] - 1e-6 || q_[i][j] > kUpperLimits[j] + 1e-6) {
          RCLCPP_WARN(
            get_logger(),
            "trajectory sample %zu, joint %zu is outside the Panda position limits "
            "(%.4f rad, limits [%.4f, %.4f]).",
            i, j + 1, q_[i][j], kLowerLimits[j], kUpperLimits[j]);
          break;
        }
      }
    }

    const double duration = static_cast<double>(q_.size() - 1) * sample_period_;
    RCLCPP_INFO(
      get_logger(), "Loaded %zu samples (%.3f s at %.3f s/sample).", q_.size(), duration,
      sample_period_);
  }

  // ── Trajectory construction ───────────────────────────────────────────────

  void append_point(
    trajectory_msgs::msg::JointTrajectory & trajectory, double time_from_start,
    const JointArray & positions, const JointArray & velocities,
    const JointArray & accelerations) const
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(positions.begin(), positions.end());
    point.velocities.assign(velocities.begin(), velocities.end());
    point.accelerations.assign(accelerations.begin(), accelerations.end());
    set_point_duration(point, time_from_start);
    trajectory.points.push_back(std::move(point));
  }

  trajectory_msgs::msg::JointTrajectory build_trajectory(const JointArray & start) const
  {
    if (q_.empty()) {
      throw std::runtime_error("trajectory is empty");
    }

    const JointArray first = q_.front();
    const JointArray start_delta = subtract(first, start);
    const double start_error = max_abs(start_delta);

    if (approach_duration_ <= 0.0 && start_error > start_tolerance_) {
      throw std::runtime_error(
        "the robot is " + std::to_string(start_error) +
        " rad away from the first trajectory point (start_tolerance = " +
        std::to_string(start_tolerance_) +
        " rad). Move the robot closer, or set approach_duration > 0 to let the "
        "sender move it there.");
    }
    if (approach_duration_ > 0.0 && start_error > max_approach_distance_) {
      throw std::runtime_error(
        "the robot is " + std::to_string(start_error) +
        " rad away from the first trajectory point, which exceeds "
        "max_approach_distance = " + std::to_string(max_approach_distance_) +
        " rad. Move the robot closer to the start pose, or raise "
        "max_approach_distance deliberately if the path is known to be safe.");
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names_;
    // A zero stamp tells JointTrajectoryController to start on its first sample.
    trajectory.header.stamp.sec = 0;
    trajectory.header.stamp.nanosec = 0;

    const double approach_duration = approach_duration_;
    if (approach_duration > 0.0) {
      const std::size_t approach_points =
        static_cast<std::size_t>(std::max(1.0, std::ceil(approach_duration / approach_dt_)));
      trajectory.points.reserve(approach_points + q_.size());

      // Quintic s(u) = 10u^3 - 15u^4 + 6u^5 with zero velocity and
      // acceleration at both ends.  The last approach sample is deliberately
      // at u < 1 so that its timestamp is strictly smaller than the first
      // dense sample at t = approach_duration.
      for (std::size_t k = 0; k < approach_points; ++k) {
        const double u = static_cast<double>(k) / static_cast<double>(approach_points);
        const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        const double ds =
          u * u * (30.0 + u * (-60.0 + 30.0 * u)) / approach_duration;
        const double dds =
          u * (60.0 + u * (-180.0 + 120.0 * u)) / (approach_duration * approach_duration);

        const JointArray positions = add_scaled(start, start_delta, s);
        const JointArray velocities = scaled(start_delta, ds);
        const JointArray accelerations = scaled(start_delta, dds);
        append_point(
          trajectory, u * approach_duration, positions, velocities, accelerations);
      }
    } else {
      trajectory.points.reserve(q_.size());
    }

    for (std::size_t i = 0; i < q_.size(); ++i) {
      const double t =
        approach_duration + static_cast<double>(i) * sample_period_ * time_scale_;
      // Scaling time by time_scale > 1 slows the motion down; velocities and
      // accelerations have to be scaled accordingly.
      const JointArray velocities = scaled(qd_[i], 1.0 / time_scale_);
      const JointArray accelerations =
        scaled(qdd_[i], 1.0 / (time_scale_ * time_scale_));
      append_point(trajectory, t, q_[i], velocities, accelerations);
    }

    return trajectory;
  }

  void log_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    for (const auto & point : trajectory.points) {
      for (double value : point.velocities) {
        max_velocity = std::max(max_velocity, std::abs(value));
      }
      for (double value : point.accelerations) {
        max_acceleration = std::max(max_acceleration, std::abs(value));
      }
    }

    const auto & last = trajectory.points.back().time_from_start;
    const double duration = static_cast<double>(last.sec) + 1.0e-9 * last.nanosec;
    RCLCPP_INFO(
      get_logger(),
      "Built trajectory: %zu points, %.3f s, max |qd| %.3f rad/s, max |qdd| %.3f rad/s^2.",
      trajectory.points.size(), duration, max_velocity, max_acceleration);
  }

  // ── ROS callbacks ─────────────────────────────────────────────────────────

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    JointArray positions{};
    bool complete = true;

    for (std::size_t j = 0; j < kNumJoints; ++j) {
      const auto iterator =
        std::find(message->name.begin(), message->name.end(), joint_names_[j]);
      if (iterator == message->name.end()) {
        complete = false;
        break;
      }
      const auto index = static_cast<std::size_t>(
        std::distance(message->name.begin(), iterator));
      if (index >= message->position.size()) {
        complete = false;
        break;
      }
      positions[j] = message->position[index];
    }

    if (!complete) {
      return;
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

  void readiness_timer_callback()
  {
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
      return;
    }

    start_attempted_ = true;
    send_trajectory();
    readiness_timer_->cancel();
  }

  void start_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!load_error_.empty()) {
      response->success = false;
      response->message = "trajectory could not be loaded: " + load_error_;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_joint_state_) {
        response->success = false;
        response->message =
          "no complete joint state received on '" + joint_state_topic_ + "' yet";
        return;
      }
    }

    if (!dry_run_ && !action_client_->action_server_is_ready()) {
      response->success = false;
      response->message =
        "action server '" + controller_action_ + "' is not available";
      return;
    }

    const bool ok = send_trajectory();
    response->success = ok;
    response->message = ok ? "trajectory sent" : "trajectory was not sent, see the log";
  }

  void cancel_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!current_goal_) {
      response->success = false;
      response->message = "no active trajectory goal";
      return;
    }

    action_client_->async_cancel_goal(current_goal_);
    RCLCPP_WARN(get_logger(), "Requested cancellation of the active trajectory goal.");
    response->success = true;
    response->message = "cancel request sent";
  }

  // ── Sending ───────────────────────────────────────────────────────────────

  bool send_trajectory()
  {
    if (!load_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "Cannot send trajectory: %s", load_error_.c_str());
      return false;
    }
    if (sending_) {
      RCLCPP_WARN(get_logger(), "A trajectory is already being sent.");
      return false;
    }

    JointArray start{};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_joint_state_) {
        RCLCPP_ERROR(get_logger(), "Cannot send trajectory: no joint state received.");
        return false;
      }
      start = current_positions_;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    try {
      trajectory = build_trajectory(start);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "Cannot build trajectory: %s", exception.what());
      return false;
    }

    log_trajectory(trajectory);

    if (dry_run_) {
      RCLCPP_INFO(
        get_logger(),
        "dry_run is set: the trajectory was built but will not be sent to the robot.");
      return true;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = std::move(trajectory);

    auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Trajectory goal was rejected by the controller.");
          sending_ = false;
          return;
        }
        current_goal_ = goal_handle;
        RCLCPP_INFO(
          get_logger(), "Trajectory goal accepted; the robot is now executing it.");
      };

    options.feedback_callback =
      [this](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        const auto current_time = now();
        if ((current_time - last_feedback_time_).seconds() < 1.0) {
          return;
        }
        last_feedback_time_ = current_time;

        double max_error = 0.0;
        for (double value : feedback->error.positions) {
          max_error = std::max(max_error, std::abs(value));
        }
        RCLCPP_INFO(get_logger(), "Tracking max |position error| = %.4f rad.", max_error);
      };

    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        sending_ = false;
        current_goal_.reset();
        const auto code = result.result ? result.result->error_code : -999;
        const auto message = result.result ? result.result->error_string : "no result";
        RCLCPP_INFO(
          get_logger(), "Trajectory finished with code %s (%d): %s",
          result_code_to_string(code).c_str(), static_cast<int>(code), message.c_str());
      };

    sending_ = true;
    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(
      get_logger(), "Sent trajectory goal to '%s'.", controller_action_.c_str());
    return true;
  }

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string trajectory_dir_;
  std::string trajectory_file_;
  std::vector<std::string> joint_names_;
  double sample_period_{0.001};
  double time_scale_{1.0};
  double approach_duration_{3.0};
  double approach_dt_{0.01};
  double max_approach_distance_{0.5};
  double start_tolerance_{0.15};
  double start_delay_{1.0};
  bool auto_start_{true};
  bool dry_run_{false};
  std::string controller_action_;
  std::string joint_state_topic_;

  // ── Trajectory data ───────────────────────────────────────────────────────
  std::vector<JointArray> q_;
  std::vector<JointArray> qd_;
  std::vector<JointArray> qdd_;
  std::string load_error_;

  // ── State ─────────────────────────────────────────────────────────────────
  std::mutex state_mutex_;
  JointArray current_positions_{};
  bool have_joint_state_{false};
  bool have_first_joint_state_{false};
  rclcpp::Time first_joint_state_time_;
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};

  bool start_attempted_{false};
  bool sending_{false};
  GoalHandle::SharedPtr current_goal_;

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr readiness_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Test1TrajectorySender>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("test1_trajectory_sender"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
