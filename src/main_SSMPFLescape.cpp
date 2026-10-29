/*
░█▄█░█▀█░▀█▀░█▀█░░░░░█▀▀░█▀▀░█▄█░█▀█░█▀▀░█░░░█▀▀░█▀▀░█▀▀░█▀█░█▀█░█▀▀
░█░█░█▀█░░█░░█░█░░░░░▀▀█░▀▀█░█░█░█▀▀░█▀▀░█░░░█▀▀░▀▀█░█░░░█▀█░█▀▀░█▀▀
░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀░░░▀░░░▀▀▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀░░░▀▀▀

main_SSMPFLescape.cpp - SSM+PFL with escape main: solves the QP at each step to re-plan the robot motion away from the human.
*/

#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <charconv>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <omp.h>
#include <Eigen/Dense>

#include "trajectory_utils.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include "robot_model.hpp"
#include "min_distance_calculation.hpp"
#include "project_root.hpp"
#include "robot_capsules.hpp"
#include "logs.hpp"
#include "core.hpp"


// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt = 1.0 / rate_hz;
const double stopping_time = 0.25;
const double velocity_PFL = 0.4;
const double pause_after_collision = 0.5;
std::vector<double> rh = {{0.16, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.1, 0.1, 0.16, 0.15}};
const double rv = 0.085;


// ─────────────────────────────────────────────────────────────────────────────
// Euclidean distance from point p to the segment a-b
// ─────────────────────────────────────────────────────────────────────────────
static double PointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                                    const Eigen::Vector3d& b) {
    const Eigen::Vector3d ab = b - a;
    const double len2 = ab.squaredNorm();
    double t = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

// ─────────────────────────────────────────────────────────────────────────────
// Return true when clearance between the obstacle point and the robot links is
// maintained along the given configuration
// ─────────────────────────────────────────────────────────────────────────────
bool CollisionFree(const RobotModel& robot, const Eigen::VectorXd& q,
                    const Eigen::Vector3d& p, double clearance) {
    static const char* kLinks[] = {"panda_link1", "panda_link2", "panda_link3",
                                    "panda_link4", "panda_link5", "panda_link6",
                                    "panda_link7", "panda_link8"};
    robot.ComputeFK(q);
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();
    for (const char* name : kLinks) {
        const Eigen::Vector3d cur = robot.GetJointPose(name).translation();
        if (PointSegmentDistance(p, prev, cur) < clearance) return false;
        prev = cur;
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Receive the merged skeleton published by data_merging.py
//
// The receive blocks until a message arrives; an empty vector is returned
// only if the message carries no skeleton at all.
// ─────────────────────────────────────────────────────────────────────────────
static std::vector<Eigen::Vector3d> receiveSkeleton(DataTransmitter& tx) {
    const std::vector<nlohmann::json> data = tx.receive_data();
    if (data.empty()) return {};
    return json_to_keypoints(data[0]);
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

    // The trajectory is read from src/trajectories/test<N>/: the waypoints can
    // be given in Cartesian coordinates (p_ref.csv) or in joint coordinates
    // (q_ref.csv), and the cumulative times (t_ref.csv) give the duration of
    // every segment.
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

    // ── PFL, SSM and escape trajectory simulation ────────────────────────────────
    Trajectory real_traj;
    real_traj.q.push_back(nominal_traj.q[0]);
    real_traj.qd.push_back(nominal_traj.qd[0]);
    real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
    real_traj.p.push_back(nominal_traj.p[0]);
    real_traj.pd.push_back(nominal_traj.pd[0]);

    bool collision = false;
    int collision_counter = 0;
    int r = 1;
    int kNumKeypoints = 11;

    // The first frame is awaited here, so that the velocity of the obstacles is
    // computed between two consecutive frames from the very first step.
    // The first frame may take a while to arrive (the vision pipeline has to
    // start up), the following ones only have to keep the stream alive.
    transmitters[0]->set_receive_timeout(10000);
    std::vector<Eigen::Vector3d> skeleton = receiveSkeleton(*transmitters[0]);
    if (skeleton.empty()) {
        throw std::runtime_error(
            "No merged skeleton received: is scripts/data_merging.py running?");
    }
    transmitters[0]->set_receive_timeout(1000);
    std::vector<Eigen::Vector3d> skeleton_prev = skeleton;

    // ── HR clearance calculation ─────────────────────────────────────────────────
    std::vector<double> HR_clearance(rh.size());
    std::transform(rh.begin(), rh.end(), HR_clearance.begin(),
                [rv](double a) { return a + rv; });


    // ── Logging ──────────────────────────────────────────────────────────────────
    const std::filesystem::path logs_dir = root / "logs" / "SSMPFLescape";
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

    std::thread logger_thread(loggerThread, std::ref(outfile_q), std::ref(outfile_qd), std::ref(outfile_qdd),
                                std::ref(outfile_p), std::ref(outfile_pd));

    // ── Starting main loop ───────────────────────────────────────────────────────
    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();
    const int n_steps = static_cast<int>(nominal_traj.t.size());
    for (int i = 0; i <= n_steps-1; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        const std::vector<Eigen::Vector3d> skeleton_new = receiveSkeleton(*transmitters[0]);
        if (!skeleton_new.empty()) {
            skeleton = skeleton_new;
        }

        // Index of the sample of the nominal trajectory that is being tracked.
        // It is clamped, so that the last sample is never read out of range.
        const size_t r_idx = std::min<size_t>(static_cast<size_t>(std::max(r, 0)),
                                               nominal_traj.q.size() - 1);

        // Distance between the human and the robot at the current
        // configuration: it drives the collision flag and it is published to
        // the web interface.

    
        // ── Construction of obstacle and velocity term vectors ───────────────────────
        std::vector<Eigen::Vector3d> ro_vec;
        std::vector<Eigen::Vector3d> vo_vec;
        std::vector<double> delta_vec;

        // A frame can hold fewer keypoints than expected: only the ones
        // that are present in both the current and the previous frame are
        // used as obstacles.
        const int n_keypoints = std::min<int>(kNumKeypoints,
                            std::min<int>(skeleton.size(), skeleton_prev.size()));
        for (int j = 0; j < n_keypoints; j++) {
            if (skeleton[j].hasNaN() || skeleton_prev[j].hasNaN()) {
                continue;
            }
            ro_vec.push_back(skeleton[j]);
            const Eigen::Vector3d v_o = (skeleton[j] - skeleton_prev[j]) / dt;
            vo_vec.push_back(v_o);
            delta_vec.push_back(-(-(HR_clearance[j] + v_o.norm() * stopping_time) / stopping_time + velocity_PFL) * stopping_time);
        }

        // ── Core SSM, PFL and escape trajectory computation ──────────────────────────
        SSMPFLResult res = SSMPFLescape(robot, dt, stopping_time, real_traj.q[i], real_traj.qd[i],
                                nominal_traj.p[r_idx], nominal_traj.pd[r_idx], nominal_traj.q[r_idx],
                                ro_vec, vo_vec, delta_vec);

        if (res.exitflag) {
            const std::optional<DistanceResult> dist =
                human_to_robot_distance(skeleton, robot, real_traj.q[i]);
            if (dist && dist->length <= HR_clearance[dist->ind_h]) {
                collision = true;
            }
            else collision = false;
        }
        else {
            collision = true;
        }

        // ── Real trajectory update ──────────────────────────────────────
        if (!collision) {
            real_traj.q.push_back(res.q_next);
            real_traj.qd.push_back(res.qd_next);
            real_traj.qdd.push_back(res.qdd_next);
            real_traj.p.push_back(res.p_next);
            real_traj.pd.push_back(res.pd_next);

            r++;  
        }
        else {
            real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
            real_traj.qd.push_back(Eigen::VectorXd::Zero(7));
            const int prev = i > 0 ? i - 1 : i;
            real_traj.q.push_back(real_traj.q[prev]);
            real_traj.p.push_back(real_traj.p[prev]);
            real_traj.pd.push_back(Eigen::Vector3d::Zero());
            if (collision_counter++ > pause_after_collision * rate_hz) {
                collision = false;
                collision_counter = 0;
            }
            // collision = false;
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
        log_queue.push(s);

        skeleton_prev = skeleton;

        g_t_log += dt;

        // ── Publish robot state, obstacle distance and trajectory ───────
        const Eigen::VectorXd& q_now = real_traj.q[i + 1];
        const std::array<std::array<double, 3>, 8> caps = robotCapsulePoints(robot, q_now);

        nlohmann::json points = nlohmann::json::array();
        for (const auto& p : caps) {
            points.push_back(nlohmann::json::array({p[0], p[1], p[2]}));
        }
        nlohmann::json radii = nlohmann::json::array();
        for (const double radius : ROBOT_CAPSULE_RADII) {
            radii.push_back(radius);
        }
        nlohmann::json q_json = nlohmann::json::array();
        for (int j = 0; j < q_now.size(); ++j) q_json.push_back(q_now(j));

        std::vector<nlohmann::json> payload;
        payload.push_back(points);
        payload.push_back(q_json);
        payload.push_back(radii);
        transmitters[1]->send_data(payload);

        // The trajectory topic carries the position of the end effector and
        // the position it is heading to, plus the collision flag
        payload.clear();
        payload.push_back(std::vector<double>(real_traj.p[i + 1].data(), real_traj.p[i + 1].data() + real_traj.p[i + 1].size()));
        payload.push_back(std::vector<double>(nominal_traj.p[r_idx].data(), nominal_traj.p[r_idx].data() + nominal_traj.p[r_idx].size()));
        payload.push_back(collision);
        transmitters[2]->send_data(payload);

        next_time += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next_time);
    }

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
