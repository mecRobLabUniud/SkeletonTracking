/*
robot_waypoint_bridge.cpp - Real-time Waypoint streamer for the Franka Panda.

Subscribes to the WAYPOINT_15 ZeroMQ stream published by main_SSM.cpp and
plays it on the real robot with libfranka at 1 kHz.

Data flow:
    main_SSM.cpp --(ZMQ PUB, WAYPOINT_15, 60 Hz)--> this process
                 --(libfranka JointPositions, 1 kHz)--> Panda

The received Waypoints carry joint positions q (and qd/qdd/p/pd/timestamp).
The bridge buffers a small amount of them, interpolates q at the 1 kHz control
rate, and rate-limits the result (velocity + acceleration) before sending it.
A watchdog smoothly stops the motion when the stream disappears.

Safety defaults:
  * the robot holds its current pose until the first Waypoint arrives;
  * the run is refused if the first Waypoint is farther than
    --max-approach-distance from the current pose;
  * velocities/accelerations are clamped to the Panda limits;
  * --dry-run receives and prints Waypoints without touching the robot.

Use --dry-run first, then test on hardware with the enabling device in reach.
*/

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "data_transmitter.hpp"
#include "waypoint_zmq.hpp"

namespace {

using Vec7 = Eigen::Matrix<double, 7, 1>;
using Clock = std::chrono::steady_clock;

constexpr size_t kJoints = 7;
constexpr double kNominalDt = 1.0 / 1000.0;

// Franka Panda joint position limits (rad).
constexpr std::array<double, kJoints> kQMin = {
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr std::array<double, kJoints> kQMax = {
     2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

std::atomic<bool> g_stop{false};

void onSignal(int) {
    g_stop.store(true);
}

double secondsSince(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

// ─────────────────────────────────────────────────────────────────────────────
// Command-line options
// ─────────────────────────────────────────────────────────────────────────────
struct Options {
    std::string robot_ip = "172.16.0.2";
    std::string zmq_address = "localhost";
    int zmq_port = waypoint_zmq::kBasePort;
    int device_id = waypoint_zmq::kDeviceId;

    bool dry_run = false;
    bool enforce_realtime = true;

    double max_velocity = 2.0;         // rad/s, per joint
    double max_acceleration = 10.0;    // rad/s^2, per joint
    double max_approach_distance = 0.5;  // rad, max |q_wp - q_current| at start
    double tracking_gain = 10.0;       // 1/s, position error -> velocity
    double buffer_time = 0.10;         // s of Waypoints buffered before playback
    int watchdog_ms = 250;             // stop if no Waypoint for this long

    // Initial configuration to drive to before starting the trajectory.
    // Empty (have_init_q == false) keeps the robot at its current pose.
    bool have_init_q = false;
    std::array<double, kJoints> init_q = {};
    double init_duration = 5.0;        // s allowed to reach the initial config
    double init_threshold = 0.01;      // rad, max joint error to count as arrived
};

void printUsage(const char* program) {
    std::cout <<
        "Usage: " << program << " [options]\n"
        "\n"
        "Receives Waypoint messages from main_SSM.cpp over ZeroMQ and streams\n"
        "them to a Franka Panda with libfranka.\n"
        "\n"
        "Robot options:\n"
        "  --robot-ip <ip>              robot FCI address (default 172.16.0.2)\n"
        "  --dry-run                    receive/log only, do not connect to the robot\n"
        "  --no-realtime                do not require a real-time kernel\n"
        "  --max-velocity <rad/s>       per-joint velocity limit (default 2.0)\n"
        "  --max-acceleration <rad/s^2> per-joint acceleration limit (default 10.0)\n"
        "  --max-approach-distance <r>  refuse if the first Waypoint is farther\n"
        "                               than this from the current pose (default 0.5)\n"
        "  --tracking-gain <1/s>        position error -> velocity gain (default 10)\n"
        "  --watchdog-ms <ms>           stop if no Waypoint arrives (default 250)\n"
        "  --buffer-ms <ms>             Waypoints buffered before playback (default 100)\n"
        "\n"
        "Initial configuration (driven to before the trajectory starts):\n"
        "  --init-q q1,...,q7           initial joint positions in rad\n"
        "                               (7 comma-separated values; default: stay put)\n"
        "  --init-duration <s>          time allowed to reach the initial config\n"
        "                               (default 5.0)\n"
        "  --init-threshold <rad>       max joint error to count as arrived\n"
        "                               (default 0.01)\n"
        "\n"
        "ZeroMQ options:\n"
        "  --connect <host>             address of main_SSM (default localhost)\n"
        "  --port <port>                base port; topic uses port + 15 (default 6000)\n"
        "\n"
        "  -h, --help                   show this help\n";
}

double parseDouble(const std::string& option, const std::string& value) {
    try {
        size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result)) throw std::invalid_argument("");
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error(option + ": expected a finite number, got '" + value + "'");
    }
}

int parseInt(const std::string& option, const std::string& value) {
    try {
        size_t used = 0;
        const int result = std::stoi(value, &used);
        if (used != value.size()) throw std::invalid_argument("");
        return result;
    } catch (const std::exception&) {
        throw std::runtime_error(option + ": expected an integer, got '" + value + "'");
    }
}

// Parse a comma-separated list of doubles into a std::array of size N.
template <size_t N>
std::array<double, N> parseDoubleList(const std::string& option,
                                      const std::string& value) {
    std::array<double, N> out{};
    size_t start = 0;
    for (size_t i = 0; i < N; ++i) {
        size_t comma = value.find(',', start);
        std::string token = (comma == std::string::npos)
                                ? value.substr(start)
                                : value.substr(start, comma - start);
        try {
            size_t used = 0;
            out[i] = std::stod(token, &used);
            if (used != token.size() || !std::isfinite(out[i]))
                throw std::invalid_argument("");
        } catch (const std::exception&) {
            throw std::runtime_error(option + ": expected a finite number, got '" +
                                     token + "'");
        }
        if (comma == std::string::npos) {
            if (i != N - 1)
                throw std::runtime_error(option + ": expected " + std::to_string(N) +
                                         " comma-separated values");
            start = value.size();
            break;
        }
        start = comma + 1;
    }
    // Reject trailing extra values.
    if (start < value.size())
        throw std::runtime_error(option + ": expected " + std::to_string(N) +
                                 " comma-separated values, got more");
    return out;
}

Options parseOptions(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string(name) + " requires a value");
            return argv[++i];
        };

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--robot-ip") {
            opt.robot_ip = value("--robot-ip");
        } else if (arg == "--connect") {
            opt.zmq_address = value("--connect");
        } else if (arg == "--port") {
            opt.zmq_port = parseInt(arg, value("--port"));
        } else if (arg == "--dry-run") {
            opt.dry_run = true;
        } else if (arg == "--no-realtime") {
            opt.enforce_realtime = false;
        } else if (arg == "--max-velocity") {
            opt.max_velocity = parseDouble(arg, value("--max-velocity"));
        } else if (arg == "--max-acceleration") {
            opt.max_acceleration = parseDouble(arg, value("--max-acceleration"));
        } else if (arg == "--max-approach-distance") {
            opt.max_approach_distance = parseDouble(arg, value("--max-approach-distance"));
        } else if (arg == "--tracking-gain") {
            opt.tracking_gain = parseDouble(arg, value("--tracking-gain"));
        } else if (arg == "--watchdog-ms") {
            opt.watchdog_ms = parseInt(arg, value("--watchdog-ms"));
        } else if (arg == "--buffer-ms") {
            opt.buffer_time = parseInt(arg, value("--buffer-ms")) / 1000.0;
        } else if (arg == "--init-q") {
            opt.init_q = parseDoubleList<kJoints>(arg, value("--init-q"));
            opt.have_init_q = true;
        } else if (arg == "--init-duration") {
            opt.init_duration = parseDouble(arg, value("--init-duration"));
        } else if (arg == "--init-threshold") {
            opt.init_threshold = parseDouble(arg, value("--init-threshold"));
        } else {
            throw std::runtime_error("unknown option '" + arg + "'");
        }
    }

    if (opt.max_velocity <= 0.0 || opt.max_acceleration <= 0.0) {
        throw std::runtime_error("velocity/acceleration limits must be positive");
    }
    if (opt.max_approach_distance < 0.0 || opt.tracking_gain <= 0.0) {
        throw std::runtime_error("approach distance must be >= 0 and tracking gain > 0");
    }
    if (opt.watchdog_ms <= 0 || opt.buffer_time < 0.0) {
        throw std::runtime_error("watchdog must be > 0 and buffer time >= 0");
    }
    if (opt.have_init_q) {
        if (opt.init_duration <= 0.0 || opt.init_threshold <= 0.0) {
            throw std::runtime_error(
                "init-duration and init-threshold must be positive when --init-q is set");
        }
    }
    return opt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread-safe Waypoint buffer written by the ZMQ thread and read by the
// 1 kHz control loop.  Reads use try_lock(), so the real-time callback never
// blocks on the receiver.
// ─────────────────────────────────────────────────────────────────────────────
class WaypointStream {
public:
    static constexpr size_t kCapacity = 64;

    struct Sample {
        double t = 0.0;
        Vec7 q = Vec7::Zero();
        Vec7 qd = Vec7::Zero();
    };

    struct Snapshot {
        std::array<Sample, kCapacity> samples{};
        size_t count = 0;
        double last_arrival = -1e9;
        uint64_t generation = 0;
    };

    // Called from the ZMQ receive thread. arrival_time is on the control clock.
    void push(const Sample& sample, double arrival_time) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_arrival_ = arrival_time;

        double t = sample.t;
        if (!std::isfinite(t)) {
            // Older producers may not fill the timestamp: synthesise one at the
            // nominal 60 Hz rate.
            t = have_stamp_ ? last_stamp_ + (1.0 / 60.0) : arrival_time;
        }

        if (have_stamp_) {
            if (t < last_stamp_ - 1e-6 || t - last_stamp_ > 1.0) {
                // Discontinuity (SSM restart, re-timed trajectory, or the
                // producer came back after a long pause).  Start a fresh buffer;
                // the control-loop limiter turns the jump into smooth motion.
                samples_.clear();
                ++generation_;
            } else if (t <= last_stamp_ + 1e-9) {
                // main_SSM repeats the current Waypoint while it is holding
                // ("steady" state).  Refresh it and keep the playback clock.
                if (!samples_.empty()) {
                    samples_.back().q = sample.q;
                    samples_.back().qd = sample.qd;
                }
                return;
            }
        }

        samples_.push_back({t, sample.q, sample.qd});
        last_stamp_ = t;
        have_stamp_ = true;
        while (samples_.size() > kCapacity) samples_.pop_front();
    }

    // Called from the control loop.  Returns false when the ZMQ thread held the
    // mutex this cycle; the caller then reuses its previous snapshot.
    bool snapshot(Snapshot& out) const {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return false;

        out.count = samples_.size();
        std::copy(samples_.begin(), samples_.end(), out.samples.begin());
        out.last_arrival = last_arrival_;
        out.generation = generation_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    std::deque<Sample> samples_;
    double last_stamp_ = 0.0;
    double last_arrival_ = -1e9;
    bool have_stamp_ = false;
    uint64_t generation_ = 0;
};

// Linear interpolation of q between the two bracketing Waypoints, with the
// trajectory velocities blended for a continuous feed-forward term.
void interpolate(const WaypointStream::Snapshot& snap, double t, Vec7& q, Vec7& qd) {
    if (snap.count == 0) {
        qd.setZero();
        return;
    }
    if (t <= snap.samples[0].t) {
        q = snap.samples[0].q;
        qd = snap.samples[0].qd;
        return;
    }
    const size_t last = snap.count - 1;
    if (t >= snap.samples[last].t) {
        q = snap.samples[last].q;
        qd.setZero();
        return;
    }

    size_t hi = 1;
    while (hi < snap.count && snap.samples[hi].t < t) ++hi;
    const auto& a = snap.samples[hi - 1];
    const auto& b = snap.samples[hi];

    const double h = std::max(b.t - a.t, 1e-6);
    const double u = std::clamp((t - a.t) / h, 0.0, 1.0);
    q = a.q + u * (b.q - a.q);
    qd = (1.0 - u) * a.qd + u * b.qd;
}

// ─────────────────────────────────────────────────────────────────────────────
// ZMQ receive loop
// ─────────────────────────────────────────────────────────────────────────────
void receiveLoop(DataTransmitter& receiver, WaypointStream& stream,
                 Clock::time_point clock_start) {
    receiver.set_receive_timeout(200);

    size_t decoded = 0;
    size_t invalid = 0;
    double last_error_log = -1.0;

    while (!g_stop.load()) {
        std::vector<nlohmann::json> data;
        try {
            data = receiver.receive_data();
        } catch (const std::exception& e) {
            if (secondsSince(clock_start) - last_error_log > 1.0) {
                last_error_log = secondsSince(clock_start);
                std::cerr << "[bridge] ZMQ receive error: " << e.what() << "\n";
            }
            continue;
        }
        if (data.empty()) continue;

        try {
            const Waypoint wp = waypoint_zmq::decode(data[0]);
            WaypointStream::Sample sample;
            sample.t = wp.timestamp;
            sample.q = Eigen::Map<const Vec7>(wp.q.data());
            sample.qd = Eigen::Map<const Vec7>(wp.qd.data());
            stream.push(sample, secondsSince(clock_start));
            ++decoded;
        } catch (const std::exception& e) {
            if (++invalid <= 5 || invalid % 100 == 0) {
                std::cerr << "[bridge] dropping invalid Waypoint: " << e.what() << "\n";
            }
        }
    }

    std::cout << "[bridge] ZMQ receiver stopped (" << decoded << " Waypoints, "
              << invalid << " invalid).\n";
}

// ─────────────────────────────────────────────────────────────────────────────
// Dry run: consume and report the stream without connecting to the robot
// ─────────────────────────────────────────────────────────────────────────────
int runDryRun(const Options& opt) {
    const Clock::time_point clock_start = Clock::now();
    DataTransmitter receiver(DataTransmitter::Mode::Receiver, opt.device_id,
                             waypoint_zmq::kTopic, opt.zmq_port, false,
                             opt.zmq_address);

    std::cout << "[bridge] dry run: listening on tcp://" << opt.zmq_address << ":"
              << (opt.zmq_port + opt.device_id) << " (topic "
              << waypoint_zmq::kTopic << "_" << opt.device_id << ")\n";

    size_t total = 0;
    size_t window = 0;
    double last_report = 0.0;
    double last_stamp = std::numeric_limits<double>::quiet_NaN();
    double first_stamp = 0.0;
    WaypointStream::Sample last;
    bool have_last = false;

    receiver.set_receive_timeout(200);
    while (!g_stop.load()) {
        std::vector<nlohmann::json> data;
        try {
            data = receiver.receive_data();
        } catch (const std::exception& e) {
            std::cerr << "[bridge] ZMQ receive error: " << e.what() << "\n";
            continue;
        }
        if (data.empty()) continue;

        try {
            const Waypoint wp = waypoint_zmq::decode(data[0]);
            last.t = wp.timestamp;
            last.q = Eigen::Map<const Vec7>(wp.q.data());
            last.qd = Eigen::Map<const Vec7>(wp.qd.data());
            have_last = true;
            if (!std::isfinite(first_stamp) || total == 0) first_stamp = wp.timestamp;
            last_stamp = wp.timestamp;
            ++total;
            ++window;
        } catch (const std::exception& e) {
            std::cerr << "[bridge] dropping invalid Waypoint: " << e.what() << "\n";
            continue;
        }

        const double now = secondsSince(clock_start);
        if (now - last_report >= 1.0) {
            const double rate = window / (now - last_report);
            last_report = now;
            window = 0;
            std::cout << "[bridge] " << std::fixed << std::setprecision(1) << rate
                      << " Waypoints/s, t=" << std::setprecision(3) << last.t
                      << " s, q1=" << last.q(0) << " rad\n" << std::defaultfloat;
        }
    }

    std::cout << "[bridge] dry run finished: " << total << " Waypoints";
    if (have_last) {
        std::cout << ", stream time " << (last_stamp - first_stamp) << " s";
    }
    std::cout << ".\n";
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// libfranka streamer
// ─────────────────────────────────────────────────────────────────────────────
class RobotStreamer {
public:
    enum class Phase { INIT, PLAYBACK };

    RobotStreamer(const Options& opt, WaypointStream& stream, Clock::time_point clock_start)
        : opt_(opt), stream_(stream), clock_start_(clock_start) {}

    int run() {
        std::cout << "[bridge] connecting to robot at " << opt_.robot_ip << " ...\n";
        franka::Robot robot(opt_.robot_ip,
                            opt_.enforce_realtime ? franka::RealtimeConfig::kEnforce
                                                  : franka::RealtimeConfig::kIgnore);
        std::cout << "[bridge] connected. Holding the current pose until the first "
                     "Waypoint arrives.\n";

        phase_ = opt_.have_init_q ? Phase::INIT : Phase::PLAYBACK;
        have_init_q_ = opt_.have_init_q;
        init_q_ = Eigen::Map<const Vec7>(opt_.init_q.data());
        init_duration_ = opt_.init_duration;
        init_threshold_ = opt_.init_threshold;
        if (phase_ == Phase::PLAYBACK) {
            std::cout << "[bridge] no initial configuration requested; "
                         "holding current pose.\n";
        } else {
            std::cout << "[bridge] driving to initial configuration over "
                      << init_duration_ << " s ...\n";
        }

        robot.control([this](const franka::RobotState& state,
                             franka::Duration period) -> franka::JointPositions {
            return controlStep(state, period);
        });

        if (!abort_reason_.empty()) {
            std::cerr << "[bridge] stopped: " << abort_reason_ << "\n";
            return 1;
        }
        std::cout << "[bridge] motion finished cleanly.\n";
        return 0;
    }

private:
    franka::JointPositions controlStep(const franka::RobotState& state,
                                       franka::Duration period) {
        const double dt = std::clamp(period.toSec(), 1e-4, 0.02);
        const double now = secondsSince(clock_start_);
        const Vec7 q_meas = Eigen::Map<const Vec7>(state.q.data());

        if (!initialized_) {
            q_cmd_ = q_meas;
            qd_cmd_.setZero();
            initialized_ = true;
            std::cout << "[bridge] initial pose:";
            for (double q : state.q) std::cout << " " << std::fixed
                                               << std::setprecision(3) << q;
            std::cout << std::defaultfloat << "\n";
        }

        // ── Take a lock-free-ish snapshot of the incoming stream ──────────────
        WaypointStream::Snapshot snap;
        if (stream_.snapshot(snap)) {
            last_snap_ = snap;
            if (snap.generation != playback_generation_) {
                playback_generation_ = snap.generation;
                playback_started_ = false;
                playback_armed_at_ = -1.0;
            }
        }

        const double waypoint_age = now - last_snap_.last_arrival;
        const bool stale = last_snap_.last_arrival > 0.0 &&
                           waypoint_age * 1000.0 > static_cast<double>(opt_.watchdog_ms);

        // ── Desired joint position for this cycle ─────────────────────────────
        Vec7 q_des = q_cmd_;
        Vec7 qd_des = Vec7::Zero();

        const bool stopping = g_stop.load() || !abort_reason_.empty();

        // ── Phase: drive to the initial configuration before playback ─────────
        if (phase_ == Phase::INIT) {
            q_des = init_q_;
            qd_des.setZero();
            const double init_err = (q_cmd_ - init_q_).cwiseAbs().maxCoeff();
            if (!init_reached_ && init_err < init_threshold_) {
                init_reached_ = true;
                init_reached_at_ = now;
                std::cout << "[bridge] reached initial configuration\n";
            }
            if (init_reached_ && (now - init_reached_at_) >= 0.2) {
                phase_ = Phase::PLAYBACK;
                std::cout << "[bridge] initial configuration reached, "
                             "starting trajectory playback.\n";
            }
        }

        // ── One-time approach check (once playback begins) ────────────────────
        if (phase_ == Phase::PLAYBACK && !approach_checked_ && last_snap_.count > 0) {
            approach_checked_ = true;
            const double distance =
                (last_snap_.samples[0].q - q_meas).cwiseAbs().maxCoeff();
            if (distance > opt_.max_approach_distance) {
                abort_reason_ =
                    "the first Waypoint is " + std::to_string(distance) +
                    " rad away from the current pose (limit " +
                    std::to_string(opt_.max_approach_distance) +
                    " rad). Move the robot closer and restart.";
            }
        }

        // ── Decide the desired joint position (playback phase) ────────────────
        if (phase_ == Phase::PLAYBACK) {
            if (!stopping && !stale && last_snap_.count > 0) {
                if (playback_armed_at_ < 0.0) playback_armed_at_ = now;

                const double span = last_snap_.samples[last_snap_.count - 1].t -
                                    last_snap_.samples[0].t;
                if (!playback_started_ &&
                    (span >= opt_.buffer_time || (now - playback_armed_at_) >= 0.5)) {
                    playback_started_ = true;
                    playback_t_ = last_snap_.samples[0].t;
                    std::cout << "[bridge] playback started: " << last_snap_.count
                              << " buffered Waypoints (" << std::fixed
                              << std::setprecision(3) << span << " s)" << std::defaultfloat
                              << "\n";
                }

                if (playback_started_) {
                    // Re-synchronise after an upstream restart or a timestamp jump.
                    if (playback_t_ < last_snap_.samples[0].t - 0.5 ||
                        playback_t_ > last_snap_.samples[last_snap_.count - 1].t + 2.0) {
                        playback_t_ = last_snap_.samples[0].t;
                    }
                    playback_t_ += dt;
                    interpolate(last_snap_, playback_t_, q_des, qd_des);
                }
            }

            // ── Watchdog: decelerate to a stop and hold ───────────────────────
            if (stale) {
                q_des = q_cmd_;
                qd_des.setZero();
                if (now - last_watchdog_log_ >= 1.0) {
                    last_watchdog_log_ = now;
                    std::cout << "[bridge] WARNING: no Waypoint for "
                              << static_cast<int>(waypoint_age * 1000.0)
                              << " ms, holding position.\n";
                }
            }
        }

        // ── Velocity / acceleration limited tracking ──────────────────────────
        const Vec7 error = q_des - q_cmd_;
        for (size_t j = 0; j < kJoints; ++j) {
            // Velocity that still allows stopping within the remaining error.
            const double stop_cap =
                std::sqrt(std::max(0.0, 2.0 * opt_.max_acceleration * std::abs(error(j))));
            double v = qd_des(j) + opt_.tracking_gain * error(j);
            v = std::clamp(v, -stop_cap, stop_cap);
            v = std::clamp(v, -opt_.max_velocity, opt_.max_velocity);

            const double dv_max = opt_.max_acceleration * dt;
            const double dv = std::clamp(v - qd_cmd_(j), -dv_max, dv_max);
            qd_cmd_(j) = std::clamp(qd_cmd_(j) + dv, -opt_.max_velocity,
                                    opt_.max_velocity);

            const double next = q_cmd_(j) + qd_cmd_(j) * dt;
            if (next < kQMin[j] || next > kQMax[j]) qd_cmd_(j) = 0.0;
            q_cmd_(j) = std::clamp(next, kQMin[j], kQMax[j]);
        }

        if (!q_cmd_.allFinite()) {
            abort_reason_ = "non-finite joint command generated";
            q_cmd_ = q_meas;
            qd_cmd_.setZero();
        }

        // ── Finish once the motion has come to rest ───────────────────────────
        if (stopping && qd_cmd_.cwiseAbs().maxCoeff() < 1e-3) {
            if (++finish_cycles_ > 100) {  // 100 ms at rest
                std::array<double, kJoints> q_out{};
                for (size_t j = 0; j < kJoints; ++j) q_out[j] = q_cmd_(j);
                return franka::MotionFinished(franka::JointPositions(q_out));
            }
        } else {
            finish_cycles_ = 0;
        }

        // ── 1 Hz status line ──────────────────────────────────────────────────
        if (now - last_status_log_ >= 1.0) {
            last_status_log_ = now;
            std::cout << "[bridge] age " << static_cast<int>(waypoint_age * 1000.0)
                      << " ms, |q_des-q_meas|max "
                      << (q_des - q_meas).cwiseAbs().maxCoeff()
                      << " rad, |qd|max " << qd_cmd_.cwiseAbs().maxCoeff()
                      << " rad/s" << (stale ? "  [STALE]" : "") << "\n";
        }

        std::array<double, kJoints> q_out{};
        for (size_t j = 0; j < kJoints; ++j) q_out[j] = q_cmd_(j);
        return franka::JointPositions(q_out);
    }

    const Options& opt_;
    WaypointStream& stream_;
    Clock::time_point clock_start_;

    bool initialized_ = false;
    bool approach_checked_ = false;
    bool playback_started_ = false;
    double playback_armed_at_ = -1.0;
    double playback_t_ = 0.0;
    uint64_t playback_generation_ = 0;

    // Initial-configuration drive phase
    Phase phase_ = Phase::INIT;
    bool have_init_q_ = false;
    Vec7 init_q_ = Vec7::Zero();
    double init_duration_ = 5.0;
    double init_threshold_ = 0.01;
    bool init_reached_ = false;
    double init_reached_at_ = -1.0;

    Vec7 q_cmd_ = Vec7::Zero();
    Vec7 qd_cmd_ = Vec7::Zero();

    WaypointStream::Snapshot last_snap_{};
    std::string abort_reason_;
    int finish_cycles_ = 0;

    double last_status_log_ = -1e9;
    double last_watchdog_log_ = -1e9;
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    Options opt;
    try {
        opt = parseOptions(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (opt.dry_run) {
        try {
            return runDryRun(opt);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            return 1;
        }
    }

    const Clock::time_point clock_start = Clock::now();
    WaypointStream stream;
    int result = 0;

    try {
        DataTransmitter receiver(DataTransmitter::Mode::Receiver, opt.device_id,
                                 waypoint_zmq::kTopic, opt.zmq_port, false,
                                 opt.zmq_address);
        std::cout << "[bridge] listening on tcp://" << opt.zmq_address << ":"
                  << (opt.zmq_port + opt.device_id) << " (topic "
                  << waypoint_zmq::kTopic << "_" << opt.device_id << ")\n";

        std::thread receiver_thread(receiveLoop, std::ref(receiver), std::ref(stream),
                                    clock_start);

        try {
            RobotStreamer streamer(opt, stream, clock_start);
            result = streamer.run();
        } catch (const franka::Exception& e) {
            std::cerr << "[bridge] libfranka error: " << e.what() << "\n";
            result = 1;
        }

        g_stop.store(true);
        receiver_thread.join();
    } catch (const std::exception& e) {
        std::cerr << "[bridge] error: " << e.what() << "\n";
        result = 1;
    }

    return result;
}
