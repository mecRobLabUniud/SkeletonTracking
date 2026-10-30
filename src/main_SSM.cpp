/*
░█▄█░█▀█░▀█▀░█▀█░░░░░█▀▀░█▀▀░█▄█
░█░█░█▀█░░█░░█░█░░░░░▀▀█░▀▀█░█░█
░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░▀

main_SSM.cpp - SSM simulation main: replays a nominal trajectory and stops while the human is closer than the safety distance at every step.
*/

#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <charconv>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <Eigen/Dense>

#include "trajectory_utils.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include "robot_model.hpp"
#include "project_root.hpp"
#include "robot_capsules.hpp"
#include "logs.hpp"
#include "core.hpp"
#include "calc_stop_duration.h"

// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt = 1.0 / rate_hz;
constexpr double WORKSPACE_RADIUS          = 0.9;  // meters, from robot base
constexpr double ROBOT_ACTIVE_VEL_THRESH   = 0.05;  // rad/s (joint velocity norm)
constexpr double IDLE_DEBOUNCE_SEC         = 0.2;   // s -- a low-velocity streak shorter
                                                     // than this is treated as a normal
                                                     // waypoint-to-waypoint zero-crossing,
                                                     // not real idle time.


// ─────────────────────────────────────────────────────────────────────────────
// Commit a still-open idle streak once the robot starts moving again, or after
// the whole task has finished.
// ─────────────────────────────────────────────────────────────────────────────
static void flushIdleStreak() {
    if (g_idle_streak_duration >= IDLE_DEBOUNCE_SEC) {
        g_idle_time += g_idle_streak_duration;
    }
    g_idle_streak_duration = 0.0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write the task metrics (T-TIME / R-IDLE / C-ACT-WS / R-STOPS) to disk
// ─────────────────────────────────────────────────────────────────────────────
static void writeMetricsFile(const std::string& pathname) {
    double t_time    = g_t_log;
    double idle_time = g_idle_time;
    double conc_time = g_concurrent_ws_time;
    int    stops     = g_stop_count.load();

    double r_idle_pct   = (t_time > 0.0) ? (idle_time / t_time) * 100.0 : 0.0;
    double c_act_ws_pct = (t_time > 0.0) ? (conc_time / t_time) * 100.0 : 0.0;

    std::ofstream out(pathname + "metrics.txt");
    out << "T-TIME (s)      : " << t_time      << "\n";
    out << "R-IDLE (%)      : " << r_idle_pct   << "\n";
    out << "C-ACT-WS (%)    : " << c_act_ws_pct << "\n";
    out << "R-STOPS (count) : " << stops        << "\n";
    out.close();

    std::cout << "\n===== Task metrics =====\n";
    std::cout << "T-TIME:   " << t_time      << " s\n";
    std::cout << "R-IDLE:   " << r_idle_pct   << " %\n";
    std::cout << "C-ACT-WS: " << c_act_ws_pct << " %\n";
    std::cout << "R-STOPS:  " << stops        << "\n";
    std::cout << "Saved to " << pathname << "metrics.txt\n";
}


// ─────────────────────────────────────────────────────────────────────────────
// Receive the merged skeleton published by data_merging.py
//
// The receive blocks until a message arrives; an empty vector is returned
// only if the message carries no skeleton at all.
// ─────────────────────────────────────────────────────────────────────────────
static Skeleton receiveSkeleton(DataTransmitter& tx) {
    const std::vector<nlohmann::json> data = tx.receive_data();
    if (data.empty()) return {};
    return json_to_keypoints(data[0]);
}

// ─────────────────────────────────────────────────────────────────────────────
// True when at least one valid skeleton point lies inside the robot workspace
// ─────────────────────────────────────────────────────────────────────────────
static bool humanInWorkspace(const Skeleton& skeleton) {
    for (const auto& p : skeleton) {
        if (p.hasNaN()) continue;
        if (p.norm() < WORKSPACE_RADIUS) return true;
    }
    return false;
}


// ─────────────────────────────────────────────────────────────────────────────
// Run the simulated tracking task for the given trajectory index and working
// directory
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, const std::filesystem::path& root) {
    const std::filesystem::path urdf = root / "src" / "urdf" / "panda.urdf";
    if (!std::filesystem::exists(urdf)) {
        throw std::runtime_error("URDF not found: " + urdf.string());
    }
    RobotModel robot(urdf.string());

    // ── Data transmitters ───────────────────────────────────────────────────
    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(3);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED", 6000, false));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // ── Delay for loading the web interface ─────────────────────────────────
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // ── Nominal trajectory construction ──────────────────────────────────────
    const std::filesystem::path traj_dir =
        root / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    Trajectory nominal_traj;
    try {
        nominal_traj = load_p2p_trajectory(robot, root.string() + "/", n_traj, dt);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load the nominal trajectory from " << traj_dir << ": "
                  << e.what() << "\n";
        return 1;
    }

    if (nominal_traj.q.size() < 2 || nominal_traj.t.empty()) {
        std::cerr << "The nominal trajectory in " << traj_dir << " is empty\n";
        return 1;
    }

    // ── SSM simulation ───────────────────────────────────────────────────────
    Trajectory real_traj;
    real_traj.q.push_back(nominal_traj.q[0]);
    real_traj.qd.push_back(nominal_traj.qd[0]);
    real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
    real_traj.p.push_back(nominal_traj.p[0]);
    real_traj.pd.push_back(nominal_traj.pd[0]);

    bool collision = false;
    bool collision_prev = false;
    int r = 1;

    // The first frame is awaited here, so that the safety layer has human data
    // from the very first step. The first frame may take a while to arrive (the
    // vision pipeline has to start up), the following ones only have to keep
    // the stream alive.
    transmitters[0]->set_receive_timeout(10000);
    Skeleton skeleton = receiveSkeleton(*transmitters[0]);
    if (skeleton.empty()) {
        throw std::runtime_error(
            "No merged skeleton received: is scripts/data_merging.py running?");
    }
    transmitters[0]->set_receive_timeout(1000);

    // ── Logging ──────────────────────────────────────────────────────────────────
    const std::filesystem::path logs_dir = root / "logs" / "SSM";
    std::string pathname = logs_dir.string() + "/";
    resetMetrics();  // fresh T-TIME / R-IDLE / C-ACT-WS / R-STOPS state for this task
    {
        std::error_code ec;
        std::filesystem::create_directories(logs_dir, ec);
        if (ec) {
            std::cerr << "executeTask: WARNING could not create '" << logs_dir
                    << "': " << ec.message() << "\n";
        }
    }
    
    std::ofstream outfile_dist(pathname + "distance.xml");
    std::ofstream outfile_q(pathname + "q.xml");
    std::ofstream outfile_qd(pathname + "qd.xml");
    std::ofstream outfile_qdd(pathname + "qdd.xml");
    std::ofstream outfile_p(pathname + "p.xml");
    std::ofstream outfile_pd(pathname + "pd.xml");

    outfile_dist << "<distance>\n";
    outfile_q << "<q>\n";
    outfile_qd << "<qd>\n";
    outfile_qdd << "<qdd>\n";
    outfile_p << "<p>\n";
    outfile_pd << "<pd>\n";
    std::cout << "Outfiles initiated\n";

    std::thread logger_thread(loggerThread, std::ref(outfile_dist), std::ref(outfile_q), std::ref(outfile_qd), std::ref(outfile_qdd),
                                std::ref(outfile_p), std::ref(outfile_pd));

    // Built once: the optimiser compiles its NLP in the constructor and is then
    // only evaluated at every step.
    StopDurationOptimizer optimizer(robot);

    // ── Starting main loop ───────────────────────────────────────────────────────
    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();
    const int n_steps = static_cast<int>(nominal_traj.t.size());
    for (int i = 0; i <= n_steps-1; ++i) {
        const Skeleton skeleton_new = receiveSkeleton(*transmitters[0]);
        if (!skeleton_new.empty()) {
            skeleton = skeleton_new;
        }

        // Index of the sample of the nominal trajectory that is being tracked.
        // It is clamped, so that the last sample is never read out of range.
        const size_t r_idx = std::min<size_t>(static_cast<size_t>(std::max(r, 0)),
                                               nominal_traj.q.size() - 1);

        const Eigen::VectorXd& q_ref   = nominal_traj.q[r_idx];
        const Eigen::VectorXd& qd_ref  = nominal_traj.qd[r_idx];
        const Eigen::VectorXd& qdd_ref = nominal_traj.qdd[r_idx];

        // ── C-ACT-WS / R-IDLE bookkeeping ────────────────────────────────────
        const bool robot_active = qd_ref.norm() > ROBOT_ACTIVE_VEL_THRESH;
        if (robot_active && humanInWorkspace(skeleton)) {
            g_concurrent_ws_time += dt;
        }
        if (robot_active) {
            flushIdleStreak();
        } else {
            g_idle_streak_duration += dt;
        }

        // ── Core SSM computation ─────────────────────────────────────────────
        std::array<double, 4> radii{};
        std::optional<DistanceResult> closest = human_to_robot_distance(skeleton, robot, q_ref);
        // DistanceResult closest;
        // if (distance.has_value()) {
        //     closest = distance.value();
        // }
        // else collision = false;
        
        const double stop_duration = optimizer.solve(q_ref, qd_ref, qdd_ref);

        collision = SSM(stop_duration, q_ref, qd_ref, qdd_ref, skeleton,
                           robot, radii, *closest);

        if (collision && !collision_prev) {
            g_stop_count.fetch_add(1);  // R-STOPS: a genuine safety stop is triggered
        }
        collision_prev = collision;

        // ── Real trajectory update ──────────────────────────────────────
        if (!collision) {
            real_traj.q.push_back(q_ref);
            real_traj.qd.push_back(qd_ref);
            real_traj.qdd.push_back(qdd_ref);
            real_traj.p.push_back(nominal_traj.p[r_idx]);
            real_traj.pd.push_back(nominal_traj.pd[r_idx]);

            r++;
        }
        else {
            real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
            real_traj.qd.push_back(Eigen::VectorXd::Zero(7));
            const int prev = i > 0 ? i - 1 : i;
            real_traj.q.push_back(real_traj.q[prev]);
            real_traj.p.push_back(real_traj.p[prev]);
            real_traj.pd.push_back(Eigen::Vector3d::Zero());
        }

        LogSample s;
        s.t = g_t_log;
        for (int j = 0; j < 7; ++j) {
            s.q[j] = real_traj.q[i][j];
            s.qd[j] = real_traj.qd[i][j];
            s.qdd[j] = real_traj.qdd[i][j];
        }
        for (int j = 0; j < 3; ++j) {
            s.p[j] = real_traj.p[i][j];
            s.pd[j] = real_traj.pd[i][j];
        }
        s.distance = *closest;
        log_queue.push(s);

        g_t_log += dt;

        // ── Publish robot state, obstacle distance and trajectory ───────
        const Eigen::VectorXd& q_now = real_traj.q[i + 1];
        const std::array<std::array<double, 3>, 8> caps = robotCapsulePoints(robot, q_now);

        nlohmann::json points = nlohmann::json::array();
        for (const auto& p : caps) {
            points.push_back(nlohmann::json::array({p[0], p[1], p[2]}));
        }
        nlohmann::json radii_json = nlohmann::json::array();
        for (const double radius : ROBOT_CAPSULE_RADII) {
            radii_json.push_back(radius);
        }
        nlohmann::json q_json = nlohmann::json::array();
        for (int j = 0; j < q_now.size(); ++j) q_json.push_back(q_now(j));

        std::vector<nlohmann::json> payload;
        payload.push_back(points);
        payload.push_back(q_json);
        payload.push_back(radii_json);
        transmitters[1]->send_data(payload);

        // The trajectory topic carries the position of the end effector and
        // the position it is heading to, plus the stop flag
        payload.clear();
        payload.push_back(std::vector<double>(real_traj.p[i + 1].data(), real_traj.p[i + 1].data() + real_traj.p[i + 1].size()));
        payload.push_back(std::vector<double>(nominal_traj.p[r_idx].data(), nominal_traj.p[r_idx].data() + nominal_traj.p[r_idx].size()));
        payload.push_back(collision);
        transmitters[2]->send_data(payload);

        next_time += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next_time);
    }

    trajectory_done.store(true);
    logger_thread.join();

    outfile_dist << "</distance>\n";
    outfile_q << "</q>\n";
    outfile_qd << "</qd>\n";
    outfile_qdd << "</qdd>\n";
    outfile_p << "</p>\n";
    outfile_pd << "</pd>\n";
    outfile_dist.close();
    outfile_q.close();
    outfile_qd.close();
    outfile_qdd.close();
    outfile_p.close();
    outfile_pd.close();

    flushIdleStreak();
    writeMetricsFile(pathname);

    return 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Entry point: parse the trajectory index and working directory, then run the
// task
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // ── Command line: <trajectory index> [repository root] ──────────────────
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
                     "(usage: " << argv[0] << " <traj_index> [repo_dir])." << std::endl;
        return 1;
    }
    if (argc > 2) root_arg = argv[2];

    const std::filesystem::path root = findProjectRoot(root_arg);

    try {
        if (execute_task(n_traj, root)) return 1;
    }
    catch (const std::exception& e) {
        // Anything unexpected exits cleanly with a clear message, instead of
        // terminating the process with a core dump.
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    printf("Exiting cleanly...\n");

    return 0;
}
