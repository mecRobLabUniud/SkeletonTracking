/*
robot_controller_ros2.cpp - ROS2 (multipanda_ros2) version of robot_controller.cpp.

  1. the arm moves to the first nominal waypoint right after start-up;
  2. the ROBOT_12 stream (ZMQ, 60 Hz) is turned into a 1 kHz trajectory with
     build_p2p_traj() (receiver thread, started together with the move);
  3. the trajectory is sent to the joint_trajectory_controller as a
     FollowJointTrajectory goal: all at once when the stream ends, or in
     overlapping chunks while the stream is running (streaming:=true).

Usage:
  robot_controller_ros2 <traj_index> [repo_dir] [--ros-args -p name:=value]

Parameters (default):
  controller_action     (/panda_effort_jtc/follow_joint_trajectory)
  joint_state_topic     (/joint_states)
  joint_names           (panda_joint1..7)
  start_duration        (3.0)  s, move to the first nominal waypoint
  stream_timeout        (1.0)  s without ROBOT_12 messages = end of stream
  approach_duration     (3.0)  s, quintic from the measured pose to the trajectory
                               (skipped when the arm is already on the first point)
  approach_dt           (0.01) s, sampling of the quintic segments
  max_approach_distance (0.5)  rad, largest allowed start / approach move
  start_tolerance       (0.15) rad, used only when approach_duration <= 0
  streaming             (false)
  chunk_duration        (1.0)  s of trajectory per streamed goal
  chunk_lead            (0.3)  s, next chunk is sent this long before the end
  chunk_ramp            (0.2)  s, blend from the measured pose into each chunk
  min_chunk_duration    (0.3)  s, smallest chunk worth sending
  max_chunk_step        (0.35) rad, a chunk starting farther away is skipped
  max_ramp_velocity     (1.0)  rad/s

Services: ~/send (non streaming: send what has been received so far),
          ~/cancel (cancel the active goal and exit).
*/

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <Eigen/Dense>

#include "utils.hpp"
#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"
#include "project_root.hpp"
#include "robot_model.hpp"

using Clock      = std::chrono::steady_clock;
using JointArray = std::array<double, 7>;
using Traj       = trajectory_msgs::msg::JointTrajectory;
using FJT        = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FJT>;
using Trigger    = std_srvs::srv::Trigger;

// ── Parameters ──────────────────────────────────────────────────────────────
constexpr double kStreamDt     = 1.0 / 60.0;  // rate of the ROBOT_12 stream
constexpr double kTrajectoryDt = 0.001;       // build_p2p_traj sampling period
constexpr int    kRobotDevice  = 12;          // DataTransmitter id of ROBOT_12
const JointArray kZero{};
constexpr double kAtStart = 0.02;             // rad: already on the first point, no approach needed

struct Params {
    std::string action, joint_state_topic;
    std::vector<std::string> joints;
    double start_duration, stream_timeout, approach_duration, approach_dt,
           max_approach_distance, start_tolerance;
    bool   streaming;
    double chunk_duration, chunk_lead, chunk_ramp, min_chunk_duration,
           max_chunk_step, max_ramp_velocity;
};

Params loadParams(rclcpp::Node& node) {
    Params p;
    p.action = node.declare_parameter<std::string>(
        "controller_action", "/panda_effort_jtc/follow_joint_trajectory");
    p.joint_state_topic = node.declare_parameter<std::string>("joint_state_topic", "/joint_states");
    p.joints = node.declare_parameter<std::vector<std::string>>(
        "joint_names", {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                        "panda_joint5", "panda_joint6", "panda_joint7"});
    p.start_duration        = node.declare_parameter<double>("start_duration", 3.0);
    p.stream_timeout        = node.declare_parameter<double>("stream_timeout", 1.0);
    p.approach_duration     = node.declare_parameter<double>("approach_duration", 3.0);
    p.approach_dt           = node.declare_parameter<double>("approach_dt", 0.01);
    p.max_approach_distance = node.declare_parameter<double>("max_approach_distance", 0.5);
    p.start_tolerance       = node.declare_parameter<double>("start_tolerance", 0.15);
    p.streaming             = node.declare_parameter<bool>("streaming", false);
    p.chunk_duration        = node.declare_parameter<double>("chunk_duration", 1.0);
    p.chunk_lead            = node.declare_parameter<double>("chunk_lead", 0.3);
    p.chunk_ramp            = node.declare_parameter<double>("chunk_ramp", 0.2);
    p.min_chunk_duration    = node.declare_parameter<double>("min_chunk_duration", 0.3);
    p.max_chunk_step        = node.declare_parameter<double>("max_chunk_step", 0.35);
    p.max_ramp_velocity     = node.declare_parameter<double>("max_ramp_velocity", 1.0);

    auto require = [](bool ok, const char* what) { if (!ok) throw std::runtime_error(what); };
    require(p.joints.size() == 7, "joint_names must contain exactly 7 names");
    require(p.start_duration > 0.0, "start_duration must be greater than zero");
    require(p.stream_timeout > 0.0, "stream_timeout must be greater than zero");
    require(p.approach_duration >= 0.0, "approach_duration must be zero or positive");
    require(p.approach_dt > 0.0, "approach_dt must be greater than zero");
    require(p.max_approach_distance >= 0.0, "max_approach_distance must be zero or positive");
    require(p.start_tolerance >= 0.0, "start_tolerance must be zero or positive");
    require(p.max_ramp_velocity > 0.0, "max_ramp_velocity must be greater than zero");
    if (p.streaming) {
        require(p.chunk_duration > 0.05, "chunk_duration must be greater than 0.05 s");
        require(p.chunk_lead >= 0.0 && p.chunk_lead < p.chunk_duration,
                "chunk_lead must be in [0, chunk_duration)");
        require(p.chunk_ramp >= 0.0 && p.chunk_ramp < p.chunk_duration,
                "chunk_ramp must be in [0, chunk_duration)");
        require(p.min_chunk_duration > 0.0 && p.min_chunk_duration <= p.chunk_duration,
                "min_chunk_duration must be in (0, chunk_duration]");
        require(p.max_chunk_step > 0.0, "max_chunk_step must be greater than zero");
    }
    return p;
}

// ── Shared state (receiver thread / ROS executor / main thread) ──────────────
struct Shared {
    std::mutex state_mutex, traj_mutex;
    JointArray q{}, dq{};                 // measured joint state
    bool have_state = false;
    Trajectory real_traj;                 // accumulated by the receiver thread
    std::atomic<bool> stop{false}, stream_done{false}, send_request{false}, cancel{false};
};

// ── Joint-trajectory helpers ────────────────────────────────────────────────
JointArray toArray(const Eigen::VectorXd& v) {
    if (v.size() != 7) throw std::runtime_error("expected a 7-element joint vector");
    JointArray out{};
    for (std::size_t j = 0; j < 7; ++j) out[j] = v[static_cast<Eigen::Index>(j)];
    return out;
}

double maxAbsDiff(const JointArray& a, const JointArray& b) {
    double m = 0.0;
    for (std::size_t j = 0; j < 7; ++j) m = std::max(m, std::abs(a[j] - b[j]));
    return m;
}

void appendPoint(Traj& traj, double t, const JointArray& q, const JointArray& dq,
                 const JointArray& ddq) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(q.begin(), q.end());
    point.velocities.assign(dq.begin(), dq.end());
    point.accelerations.assign(ddq.begin(), ddq.end());
    const int64_t ns = static_cast<int64_t>(std::llround(t * 1.0e9));
    point.time_from_start.sec     = static_cast<int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    traj.points.push_back(std::move(point));
}

double lastTime(const Traj& traj) {
    if (traj.points.empty()) return -1.0;
    const auto& t = traj.points.back().time_from_start;
    return static_cast<double>(t.sec) + 1.0e-9 * t.nanosec;
}

// Samples of the quintic (p0,v0,a0) -> (p1,v1,a1) over `duration`, starting at
// t0 and excluding the end point, which the caller adds at t0 + duration.
void appendQuintic(Traj& traj, double t0, double duration, double dt,
                   const JointArray& p0, const JointArray& v0, const JointArray& a0,
                   const JointArray& p1, const JointArray& v1, const JointArray& a1) {
    if (!(duration > 0.0)) return;
    const std::size_t count = static_cast<std::size_t>(std::max(1.0, std::ceil(duration / dt)));

    std::array<JointArray, 6> c{};
    const double T1 = duration, T2 = duration * duration;
    for (std::size_t j = 0; j < 7; ++j) {
        const double dp = p1[j] - p0[j];
        c[0][j] = p0[j];
        c[1][j] = v0[j];
        c[2][j] = 0.5 * a0[j];
        c[3][j] = (20.0 * dp - (8.0 * v1[j] + 12.0 * v0[j]) * T1 -
                   (3.0 * a0[j] - a1[j]) * T2) / (2.0 * T1 * T2);
        c[4][j] = (-30.0 * dp + (14.0 * v1[j] + 16.0 * v0[j]) * T1 +
                   (3.0 * a0[j] - 2.0 * a1[j]) * T2) / (2.0 * T2 * T2);
        c[5][j] = (12.0 * dp - 6.0 * (v1[j] + v0[j]) * T1 -
                   (a0[j] - a1[j]) * T2) / (2.0 * T2 * T2 * T1);
    }

    for (std::size_t k = 0; k < count; ++k) {
        const double t = duration * static_cast<double>(k) / static_cast<double>(count);
        JointArray p{}, v{}, a{};
        for (std::size_t j = 0; j < 7; ++j) {
            p[j] = c[0][j] + t * (c[1][j] + t * (c[2][j] + t * (c[3][j] + t * (c[4][j] + t * c[5][j]))));
            v[j] = c[1][j] + t * (2.0 * c[2][j] + t * (3.0 * c[3][j] + t * (4.0 * c[4][j] + t * 5.0 * c[5][j])));
            a[j] = 2.0 * c[2][j] + t * (6.0 * c[3][j] + t * (12.0 * c[4][j] + t * 20.0 * c[5][j]));
        }
        appendPoint(traj, t0 + t, p, v, a);
    }
}

void checkDistance(double error, double limit, const char* name) {
    if (error > limit)
        throw std::runtime_error("the robot is " + std::to_string(error) +
                                 " rad away from the first trajectory point (" + name +
                                 " = " + std::to_string(limit) + " rad)");
}

// Goal = quintic lead-in from (q0, v0) to sample `target`, followed by the
// samples [target, end).  `first`: approach to samples[base]; otherwise a
// short blend into the middle of the stream.  Returns false if the chunk must
// be skipped.  `lead_out` is the duration of the lead-in.  The caller holds
// traj_mutex.
bool buildGoal(const Params& p, const Trajectory& traj, const JointArray& q0,
               const JointArray& v0, std::size_t base, std::size_t end, bool first,
               Traj& goal, double& lead_out) {
    std::size_t target = base;
    double lead = p.approach_duration;

    if (first) {
        const double error = maxAbsDiff(toArray(traj.wp[base].q), q0);
        if (p.approach_duration > 0.0) checkDistance(error, p.max_approach_distance, "max_approach_distance");
        else                           checkDistance(error, p.start_tolerance, "start_tolerance");
        if (error < kAtStart) lead = 0.0;   // already at the start: begin at once
    } else {
        // The blend replaces the first chunk_ramp seconds of the chunk; short
        // chunks (near the end of the data) get a proportionally shorter blend.
        std::size_t ramp = static_cast<std::size_t>(std::max(1.0, std::ceil(p.chunk_ramp / kTrajectoryDt)));
        const std::size_t span = end - 1 - base;
        ramp = span > 0 ? std::min(ramp, std::max<std::size_t>(1, 2 * span / 5)) : 0;
        target = base + ramp;

        const double step = maxAbsDiff(toArray(traj.wp[target].q), q0);
        if (step > p.max_chunk_step) return false;
        // Peak velocity of a quintic between two rest poses is 1.875 * step / T.
        lead = std::max(ramp > 0 ? ramp * kTrajectoryDt : p.chunk_ramp,
                        1.875 * step / p.max_ramp_velocity);
    }

    lead_out = lead;
    goal = Traj();
    goal.joint_names = p.joints;   // zero stamp: the controller starts on the first sample
    appendQuintic(goal, 0.0, lead, p.approach_dt, q0, v0, kZero, toArray(traj.wp[target].q),
                  toArray(traj.wp[target].qd), toArray(traj.wp[target].qdd));
    const double offset = lead - traj.wp[target].timestamp;
    for (std::size_t i = target; i < end; ++i) {
        const double t = offset + traj.wp[i].timestamp;
        if (t <= lastTime(goal) + 1.0e-9) continue;
        appendPoint(goal, t, toArray(traj.wp[i].q), toArray(traj.wp[i].qd), toArray(traj.wp[i].qdd));
    }
    return true;
}

// ── FollowJointTrajectory client: tracks only the newest goal ───────────────
class GoalSender {
public:
    GoalSender(const rclcpp::Node::SharedPtr& node, const std::string& action)
        : client_(rclcpp_action::create_client<FJT>(node, action)) {}

    bool ready() { return client_->action_server_is_ready(); }

    void send(Traj trajectory) {
        uint64_t gen;
        {
            std::lock_guard<std::mutex> lock(m_);
            gen = ++generation_;
            busy_ = true; accepted_ = false; ok_ = false;
            handle_.reset();
        }
        FJT::Goal goal;
        goal.trajectory = std::move(trajectory);

        auto options = rclcpp_action::Client<FJT>::SendGoalOptions();
        options.goal_response_callback = [this, gen](GoalHandle::SharedPtr handle) {
            std::lock_guard<std::mutex> lock(m_);
            if (gen != generation_) {                 // a newer goal was sent meanwhile
                if (handle) client_->async_cancel_goal(handle);
                return;
            }
            if (!handle) { busy_ = false; return; }   // rejected
            handle_ = handle;
            accepted_ = true;
        };
        options.result_callback = [this, gen](const GoalHandle::WrappedResult& result) {
            std::lock_guard<std::mutex> lock(m_);
            if (gen != generation_) return;           // result of a superseded goal
            busy_ = false;
            handle_.reset();
            ok_ = result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result &&
                  result.result->error_code == FJT::Result::SUCCESSFUL;
        };
        client_->async_send_goal(goal, options);
    }

    void cancel() {
        std::lock_guard<std::mutex> lock(m_);
        if (handle_) client_->async_cancel_goal(handle_);
    }

    bool idle()      { std::lock_guard<std::mutex> lock(m_); return !busy_; }
    bool pending()   { std::lock_guard<std::mutex> lock(m_); return busy_ && !accepted_; }
    bool succeeded() { std::lock_guard<std::mutex> lock(m_); return ok_; }

private:
    rclcpp_action::Client<FJT>::SharedPtr client_;
    std::mutex m_;
    uint64_t generation_ = 0;
    bool busy_ = false, accepted_ = false, ok_ = false;
    GoalHandle::SharedPtr handle_;
};

// Runs the ROS executor in a background thread.
struct Spinner {
    rclcpp::executors::SingleThreadedExecutor exec;
    std::thread thread;
    explicit Spinner(const rclcpp::Node::SharedPtr& node) {
        exec.add_node(node);
        thread = std::thread([this] { exec.spin(); });
    }
    ~Spinner() { exec.cancel(); thread.join(); }
};

// Stops and joins a thread on scope exit (also when an exception unwinds).
struct Joiner {
    std::thread& thread;
    std::atomic<bool>& stop;
    ~Joiner() { stop = true; if (thread.joinable()) thread.join(); }
};

template <class Cond>
bool waitUntil(Cond&& cond) {
    while (rclcpp::ok()) {
        if (cond()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

// ── ROBOT_12 stream ─────────────────────────────────────────────────────────
Waypoint receiveRobotStates(DataTransmitter& tx) {
    const std::vector<nlohmann::json> data = tx.receive_data();
    if (data.size() < 5) return {};

    Waypoint wp;
    wp.q   = json_to_robotstate(data[2]);
    wp.qd  = json_to_robotstate(data[3]);
    wp.qdd = json_to_robotstate(data[4]);
    return wp;
}

// Appends a p2p segment to real_traj for every new configuration received.
void robotStatesThread(const Params& p, const Trajectory& nominal_traj, RobotModel& robot_model,
                       Shared& s) {
    DataTransmitter receiver(DataTransmitter::Mode::Receiver, kRobotDevice, "ROBOT");
    receiver.set_receive_timeout(100);

    Waypoint previous = nominal_traj.wp.front();
    Waypoint next = previous;
    double t = 0.0;
    std::vector<double> vnorm;
    bool received = false;
    Clock::time_point last_message = Clock::now();

    while (!s.stop && rclcpp::ok()) {
        Waypoint wp;
        try {
            wp = receiveRobotStates(receiver);
        } catch (const std::exception&) {
            continue;
        }

        if (wp.q.size() == 7 && wp.qd.size() == 7 && wp.qdd.size() == 7 &&
            wp.q.allFinite() && wp.qd.allFinite() && wp.qdd.allFinite()) {
            received = true;
            last_message = Clock::now();
            if (wp.q != next.q) {
                std::lock_guard<std::mutex> lock(s.traj_mutex);
                previous = next;
                next = wp;
                build_p2p_traj(previous.q, next.q, &vnorm, s.real_traj, kStreamDt, t,
                               robot_model, kTrajectoryDt, previous.qd, previous.qdd,
                               next.qd, next.qdd);
            }
        }

        // The stream ends when no message arrives any more.
        if (received &&
            std::chrono::duration<double>(Clock::now() - last_message).count() > p.stream_timeout)
            break;
    }

    std::lock_guard<std::mutex> lock(s.traj_mutex);
    s.stream_done = true;
}

// ── Live streaming: overlapping goals of chunk_duration seconds each ────────
void streamingLoop(const Params& p, Shared& s, GoalSender& sender) {
    const std::size_t chunk = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(p.chunk_duration / kTrajectoryDt)));
    const std::size_t minimum = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::llround(p.min_chunk_duration / kTrajectoryDt)));

    bool sent = false, used_approach = false;
    std::size_t last_base = 0, last_end = 0;
    Clock::time_point send_time, next_send, last_attempt;

    while (rclcpp::ok() && !s.cancel) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const Clock::time_point now = Clock::now();
        // A newer goal is sent only after the previous one has been accepted.
        if (sent && (now < next_send || sender.pending())) continue;
        if (now - last_attempt < std::chrono::milliseconds(100)) continue;
        last_attempt = now;

        JointArray q{}, dq{};
        {
            std::lock_guard<std::mutex> lock(s.state_mutex);
            q = s.q;
            dq = s.dq;
        }

        const bool first = !sent;
        Traj goal;
        std::size_t base = 0, end = 0;
        double lead = 0.0;
        bool covers_all = false;
        {
            std::lock_guard<std::mutex> lock(s.traj_mutex);
            const std::size_t count = s.real_traj.wp.size();
            if (count == 0) {
                if (s.stream_done) throw std::runtime_error("the ROBOT_12 stream produced no trajectory samples");
                continue;
            }

            // Progress along the trajectory is purely time-based (a pose search
            // would stick to one sample during a hold).
            if (!first) {
                double elapsed = std::chrono::duration<double>(now - send_time).count();
                if (used_approach) elapsed -= p.approach_duration;
                base = last_base;
                if (elapsed > 0.0)
                    base = std::min(std::min(last_end, count - 1),
                                    base + static_cast<std::size_t>(std::llround(elapsed / kTrajectoryDt)));
            }
            if (base >= count) {
                if (s.stream_done) return;
                continue;
            }

            end = s.stream_done ? count : std::min(count, base + chunk);
            if (!s.stream_done && end - base < (first ? chunk : minimum)) continue;

            JointArray v0{};
            if (!first)
                for (std::size_t j = 0; j < 7; ++j)
                    v0[j] = std::clamp(dq[j], -p.max_ramp_velocity, p.max_ramp_velocity);

            if (!sender.ready() || !buildGoal(p, s.real_traj, q, v0, base, end, first, goal, lead)) continue;
            covers_all = (end == count);
        }

        const double duration = lastTime(goal);
        sender.send(std::move(goal));
        sent = true;
        send_time = now;
        last_base = base;
        last_end = end;
        used_approach = first && lead > 0.0;
        // No faster than min_chunk_duration, to avoid controller churn near the data frontier.
        next_send = now + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(std::max(duration - p.chunk_lead, p.min_chunk_duration)));

        if (s.stream_done && covers_all) return;
    }
}

int controller(int n_traj, const std::filesystem::path& root) {
    auto node = std::make_shared<rclcpp::Node>("robot_controller_ros2");
    const Params p = loadParams(*node);

    // ── Kinematic model ─────────────────────────────────────────────────────
    const std::filesystem::path urdf = root / "src" / "urdf" / "panda.urdf";
    if (!std::filesystem::exists(urdf)) {
        std::cerr << "URDF not found: " << urdf.string() << "\n";
        return 1;
    }
    RobotModel robot_model(urdf.string());

    // ── Nominal trajectory construction ─────────────────────────────────────
    Trajectory nominal_traj;
    try {
        nominal_traj = load_trajectory(robot_model, root.string() + "/", n_traj, kStreamDt);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load the nominal trajectory: " << e.what() << "\n";
        return 1;
    }
    if (nominal_traj.wp.empty()) {
        std::cerr << "The nominal trajectory is empty\n";
        return 1;
    }

    // ── Robot interface (joint states, FollowJointTrajectory, services) ─────
    Shared s;
    GoalSender sender(node, p.action);

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        p.joint_state_topic, rclcpp::QoS(10),
        [&](sensor_msgs::msg::JointState::SharedPtr msg) {
            JointArray q{}, dq{};
            for (std::size_t j = 0; j < 7; ++j) {
                const auto it = std::find(msg->name.begin(), msg->name.end(), p.joints[j]);
                if (it == msg->name.end()) return;
                const auto i = static_cast<std::size_t>(it - msg->name.begin());
                if (i >= msg->position.size()) return;
                q[j] = msg->position[i];
                if (i < msg->velocity.size()) dq[j] = msg->velocity[i];
            }
            std::lock_guard<std::mutex> lock(s.state_mutex);
            s.q = q;
            s.dq = dq;
            s.have_state = true;
        });
    auto send_srv = node->create_service<Trigger>(
        "~/send", [&](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
            res->success = !p.streaming;
            if (!p.streaming) s.send_request = true;
        });
    auto cancel_srv = node->create_service<Trigger>(
        "~/cancel", [&](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
            res->success = true;
            s.cancel = true;
        });

    // ── Data receiver ───────────────────────────────────────────────────────
    std::thread states_thread(robotStatesThread, std::cref(p), std::cref(nominal_traj),
                              std::ref(robot_model), std::ref(s));
    Joiner states_joiner{states_thread, s.stop};
    Spinner spinner(node);

    // ── Move to the initial joint configuration ─────────────────────────────
    if (!waitUntil([&] {
            std::lock_guard<std::mutex> lock(s.state_mutex);
            return s.have_state && sender.ready();
        }))
        return 0;

    JointArray q{};
    {
        std::lock_guard<std::mutex> lock(s.state_mutex);
        q = s.q;
    }
    const JointArray start = toArray(nominal_traj.wp.front().q);
    const double distance = maxAbsDiff(start, q);
    checkDistance(distance, p.max_approach_distance, "max_approach_distance");
    const double duration = std::max(p.start_duration, 1.875 * distance / p.max_ramp_velocity);

    Traj move;
    move.joint_names = p.joints;
    appendQuintic(move, 0.0, duration, p.approach_dt, q, kZero, kZero, start, kZero, kZero);
    appendPoint(move, duration, start, kZero, kZero);
    sender.send(std::move(move));

    if (!waitUntil([&] { return sender.idle() || s.cancel; })) return 0;
    if (!s.cancel && !sender.succeeded())
        throw std::runtime_error("the move to the initial joint configuration failed");

    // ── Follow the ROBOT_12 stream ──────────────────────────────────────────
    if (!s.cancel) {
        if (p.streaming) {
            streamingLoop(p, s, sender);
        } else if (waitUntil([&] { return s.stream_done || s.send_request || s.cancel; }) && !s.cancel) {
            {
                std::lock_guard<std::mutex> lock(s.state_mutex);
                q = s.q;
            }
            Traj goal;
            double lead = 0.0;
            {
                std::lock_guard<std::mutex> lock(s.traj_mutex);
                if (s.real_traj.wp.empty())
                    throw std::runtime_error("the ROBOT_12 stream produced no trajectory samples");
                buildGoal(p, s.real_traj, q, kZero, 0, s.real_traj.wp.size(), true, goal, lead);
            }
            sender.send(std::move(goal));
        }
    }

    // Wait for the last goal (or for ~/cancel).
    waitUntil([&] { return s.cancel || sender.idle(); });
    if (s.cancel) {
        sender.cancel();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point: parse the trajectory index and working directory, then run the
// task
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // ── Command line: <trajectory index> [repository root] [ROS args] ───────
    int n_traj = -1;
    std::string root_arg;
    if (argc > 1) {
        try {
            std::string arg(argv[1]);
            const char* begin = arg.data();
            const char* end   = arg.data() + arg.size();
            auto [ptr, ec] = std::from_chars(begin, end, n_traj);
            if (ec != std::errc{} || ptr != end || n_traj < 0)
                throw 1;
        }
        catch (int) {
            std::cerr << "Error: the trajectory index must be a valid non-negative integer."
                      << std::endl;
            return 1;
        }
    }
    else {
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
        status = controller(n_traj, root);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        status = 1;
    }

    rclcpp::shutdown();
    return status;
}