/*
░█▄█░█▀█░▀█▀░█▀█░░░░░█▀▀░█▀▀░█▄█
░█░█░█▀█░░█░░█░█░░░░░▀▀█░▀▀█░█░█
░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀░▀

main_SSM.cpp - Main executable: replays a recorded trajectory on the Panda while monitoring the human skeleton, computing the stop duration and stopping on collision.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <array>
#include <charconv>
#include <filesystem>

#include <Eigen/Dense>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "flag_stop.h"
#include "shared_state.h"
#include "calc_stop_duration.h"
#include "skeleton_reader.h"
#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"       // Trajectory, load_p2p_trajectory
#include "trajectory_publisher.hpp"   // PlaybackState, limitTorqueRate, TrajectoryPublisher
#include "robot_model.hpp"           // RobotModel wrapper around pinocchio
#include "logs.hpp"

// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt = 1.0 / rate_hz;
constexpr double WORKSPACE_RADIUS          = 0.9;  // meters, from robot base
constexpr double ROBOT_ACTIVE_VEL_THRESH   = 0.05;  // rad/s (joint velocity norm)
constexpr double IDLE_DEBOUNCE_SEC         = 0.2;   // s -- a low-velocity streak shorter
                                                     // than this is treated as a normal
                                                     // waypoint-to-waypoint zero-crossing,
                                                     // not real idle time. Tune to be longer
                                                     // than the deceleration/acceleration
                                                     // blend time around your waypoints.
// Grace period before a missing/stale skeleton stream forces a safety stop
// (s): absorbs the startup latency of the vision pipeline, so the robot does
// not trip the safety layer before the first frames arrive.
constexpr double STALE_STOP_GRACE_S        = 1.0;
// Maximum time (s) the robot stays stopped waiting for the obstacle to clear
// before it resumes the trajectory anyway.
constexpr double MAX_STOP_WAIT_S            = 10.0;

// Call once after the whole task (all waypoints/segments) has finished, to
// commit a still-open idle streak (e.g. the robot's final resting period)
// that hasn't yet been flushed by a subsequent "velocity above threshold" cycle.
static void flushIdleStreak() {
    if (g_idle_streak_duration >= IDLE_DEBOUNCE_SEC) {
        g_idle_time += g_idle_streak_duration;
    }
    g_idle_streak_duration = 0.0;
}

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

// start configuration

inline constexpr std::array<double, 7> Q_HOME = {
    -0.3180,
    0.4110,
    -0.2590,
    -1.5090,
    0.1200,
    1.9870,
    0.2620
};

void collisionCheckerThread(RobotModel& robot, std::ofstream& outfile_dist,
                             TrajectoryPublisher& publisher) {
    StopDurationOptimizer optimizer(robot);  // builds solver once here
    // Ensure FK/data are up to date for downstream consumers
    Eigen::VectorXd q_home_init(7);
    for (int j = 0; j < 7; ++j) q_home_init(j) = Q_HOME[j];
    robot.ComputeFK(q_home_init);

    auto t_i = std::chrono::steady_clock::now();  // task start, for logging
    auto last_metrics_t = t_i;                    // for C-ACT-WS dt integration
    auto last_tick_t    = t_i;                    // for the stale-data timer

    // Last computed closest-point pair / stop decision, kept so the last known
    // values keep being published to web_interface.py even on ticks where the
    // skeleton is empty (NaN -> JSON null -> None on the Python side)
    Eigen::Vector3d last_C_h = Eigen::Vector3d::Constant(NAN);
    Eigen::Vector3d last_C_r = Eigen::Vector3d::Constant(NAN);
    bool last_collision = false;

    // How long the skeleton data has been missing or stale
    double stale_elapsed = 0.0;

    while (!trajectory_done.load()) {
        auto now = std::chrono::steady_clock::now();
        double dt_loop = std::chrono::duration<double>(now - last_tick_t).count();
        last_tick_t = now;

        // Read current robot state
        std::array<double, 7> q_arr, qp_arr, qpp_arr;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex);
            q_arr   = shared_robot_state.q;
            qp_arr  = shared_robot_state.q_p;
            qpp_arr = shared_robot_state.q_pp;
        }

        // Convert to Eigen
        Eigen::VectorXd q   = Eigen::Map<Eigen::VectorXd>(q_arr.data(),   7);
        Eigen::VectorXd q_p = Eigen::Map<Eigen::VectorXd>(qp_arr.data(),  7);
        Eigen::VectorXd q_pp= Eigen::Map<Eigen::VectorXd>(qpp_arr.data(), 7);

        // Run optimization
        double stop_duration = optimizer.solve(q, q_p, q_pp);
        shared_stop_duration.store(stop_duration);

        // Read skeleton
        Skeleton skeleton;
        {
            std::lock_guard<std::mutex> lock(skeleton_mutex);
            skeleton = shared_skeleton;
        }

        // ---- C-ACT-WS: concurrent robot+human activity inside the workspace ----
        {
            auto now_metrics = std::chrono::steady_clock::now();
            double dt_conc = std::chrono::duration<double>(now_metrics - last_metrics_t).count();
            last_metrics_t = now_metrics;

            // Robot: always considered inside its own workspace; "active" = moving
            bool robot_active = q_p.norm() > ROBOT_ACTIVE_VEL_THRESH;

            // Human: active whenever any tracked point is inside the workspace
            // (presence is enough -- the human need not be moving)
            bool human_in_ws = false;
            for (const auto& pt : skeleton) {
                if (std::isnan(pt[0]) || std::isnan(pt[1]) || std::isnan(pt[2])) continue;
                double dist_from_base = std::sqrt(pt[0]*pt[0] + pt[1]*pt[1] + pt[2]*pt[2]);
                if (dist_from_base < WORKSPACE_RADIUS) {
                    //std::cout << "human is inside robot ws\n";
                    human_in_ws = true;
                    break;
                }
            }

            if (robot_active && human_in_ws) {
                //std::cout << "concurrent activity detected: " << dt_conc << "s\n";
                g_concurrent_ws_time += dt_conc;
            }
        }
/*
        std::cout << "Skeleton points (" << skeleton.size() << " total):\n";
        for (size_t i = 0; i < skeleton.size(); ++i) {
            const auto& pt = skeleton[i];
            std::cout << "  [" << i << "] -> ";
            if (std::isnan(pt[0]) && std::isnan(pt[1]) && std::isnan(pt[2])) {
                std::cout << "NaN, NaN, NaN\n";
            } else {
                std::cout << "x: " << pt[0] << ", y: " << pt[1] << ", z: " << pt[2] << "\n";
            }
        }
*/
        std::array<double, 4> radii;
        // Stop decision 
        if (!skeleton.empty() && !skeleton_stale.load()) {
            stale_elapsed = 0.0;

            DistanceResult closest;
            closest.distance = NAN;
            closest.C_h = Eigen::Vector3d::Constant(NAN);
            closest.C_r = Eigen::Vector3d::Constant(NAN);

            robot.ComputeFK(q);
            bool should_stop = flagStop(stop_duration, q, q_p, q_pp,
                                        skeleton, robot.getModel(), robot.getData(), radii, &closest);
            stop_flag.store(should_stop);
            shared_rh_distance.store(closest.distance);

            last_C_h       = closest.C_h;
            last_C_r       = closest.C_r;
            last_collision = should_stop;

            //  Log the closest robot/human points, mirroring dist_segment.xml 
            double t_exe = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_i).count();
 
            outfile_dist << "\t<keypoint time='" << t_exe << "'>\n";
            outfile_dist << "\t\t<stop_duration>" << stop_duration << "</stop_duration>\n";
            outfile_dist << "\t\t<radii>" << radii[0] << ", " << radii[1] << ", " << radii[2] << ", " << radii[3] << "</radii>\n";
            
            outfile_dist << "\t\t<position>\n";
            for (int j = 0; j < 7; ++j)
                outfile_dist << "\t\t\t<point id='" << j << "'>" << q[j] << "</point>\n";
            
            outfile_dist << "\t\t</position>\n\t\t<speed>\n";
            for (int j = 0; j < 7; ++j)
                outfile_dist << "\t\t\t<point id='" << j << "'>" << q_p[j] << "</point>\n";
            outfile_dist << "\t\t</speed>\n\t</keypoint>\n";

        } else {
            // The skeleton stream is missing or has gone stale: after a short
            // grace period (to absorb the startup latency of the vision
            // pipeline) force a stop instead of moving without human-position
            // data. The last known closest-point pair keeps being published.
            stale_elapsed += dt_loop;
            if (stale_elapsed > STALE_STOP_GRACE_S) {
                stop_flag.store(true);
            }
        }

        // Stream the closest-point pair / stop decision to web_interface.py
        publisher.publishClosestPoints(last_C_h, last_C_r, last_collision);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Play the nominal trajectory.
//
// The nominal trajectory is already sampled at the control rate: every sample
// holds the joint configuration, the velocity and the acceleration to be
// commanded, so the playback is a plain replay of those samples.
//
// The safety layer is unchanged: when the collision checker asks for a stop the
// robot brakes to a full stop, waits for the obstacle to clear and then
// resumes the nominal trajectory from the sample that was being played.
// ─────────────────────────────────────────────────────────────────────────────
void executeTrajectory (
    const Trajectory&     nominal_traj,
    TrajectoryPublisher&  publisher,
    RobotModel&           robot) {

    const double TOLL_NO_STOP = 0.05;
    bool flag_continue = true;

    const size_t n_samples = nominal_traj.q.size();

    // A trajectory with no motion carries nothing to play: skip it instead of
    // building a degenerate polynomial (which would divide by zero).
    if (n_samples < 2) {
        std::cout << "executeTrajectory: the nominal trajectory is too short, skipped\n";
        return;
    }

    const pinocchio::FrameIndex EE_ID = robot.getModel().getFrameId("panda_link8");
    pinocchio::Data& data = robot.getData();

    // The last sample of the nominal trajectory is the final configuration of
    // the task: the safety stop stays armed until the robot reaches it.
    const Eigen::VectorXd q_end = nominal_traj.q.back();

    // Index of the first sample of the nominal trajectory that is played: it is
    // moved forward when the playback resumes after a safety stop.
    size_t idx_start = 0;
    // Index of the sample that was being played when the safety stop was
    // triggered: the playback resumes from it.
    size_t idx_stop = 0;

    // Configuration the motion starts from: the first sample of the nominal
    // trajectory, or the measured one when the playback resumes after a stop.
    Eigen::VectorXd q_c = nominal_traj.q.front();

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 20.0, 20.0, 7.0}};

    enum State {RUNNING, STOPPING};     // logica a stati del movimento
    State current_state = RUNNING;

    while (flag_continue) {
        // The samples of the nominal trajectory that are still to be played.
        // They are already sampled at the control rate, so they are replayed as
        // they are and the playback lasts exactly as long as the recorded
        // motion.
        const int num_points = static_cast<int>(n_samples - idx_start);
        const double traj_duration_new =
            (nominal_traj.t.back() - nominal_traj.t[idx_start]) + TRAJ_STEP;

        Eigen::MatrixXd Q(num_points, 7);
        Eigen::MatrixXd Q_p(num_points, 7);
        Eigen::MatrixXd Q_pp(num_points, 7);

        for (int k = 0; k < num_points; ++k) {
            const Eigen::VectorXd& q_k   = nominal_traj.q[idx_start + k];
            const Eigen::VectorXd& qd_k  = nominal_traj.qd[idx_start + k];
            const Eigen::VectorXd& qdd_k = nominal_traj.qdd[idx_start + k];
            for (int j = 0; j < 7; ++j) {
                Q(k, j)    = q_k(j);
                Q_p(k, j)  = qd_k(j);
                Q_pp(k, j) = qdd_k(j);
            }
        }
 
        bool stop_flag_local = false;
        double stop_duration = 0.4;
        Eigen::VectorXd q_meas     = q_c;

        double t = 0.0;  // accumulated playback time
        // g_t_log (global) accumulates regardless of RUNNING/STOPPING, for sampling and T-TIME

        // braking variables
        double t_brake = 0.0;
        int stop_num_points = 0;
        
        Eigen::VectorXd q_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd v_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::VectorXd a_0_brake = Eigen::VectorXd::Zero(7);
        Eigen::MatrixXd Q_stop, Q_p_stop, Q_pp_stop;

        // Simulated plant state, replaces franka::RobotState: starts from the
        // currently commanded configuration and, with no robot attached,
        // tracks the commanded trajectory exactly (one tick of delay).
        PlaybackState state;
        for (int j = 0; j < 7; ++j) {
            state.q[j] = q_c(j);
        }

        // execute trajectory: the same state machine as before, stepped at the
        // old 1 kHz control rate, with every commanded sample published to
        // web_interface.py through the trajectory publisher instead of being
        // streamed to libfranka (control needs to just impose the trajectory,
        // be it normal or stopping).
        bool motion_finished = false;
        double t_publish = TRAJ_PUBLISH_PERIOD;  // publish on the very first tick

        try {
            auto next_tick = std::chrono::steady_clock::now();

            for (int tick = 0; !motion_finished; ++tick) {
                // libfranka called the control callback with period 0 on the
                // first cycle, then with the fixed 1 kHz period: mirror that
                const double period_s = (tick == 0) ? 0.0 : TRAJ_STEP;

                // Update shared robot state for collision checker 
                {
                    std::lock_guard<std::mutex> lock(robot_state_mutex);
                    for (int j = 0; j < 7; ++j) {
                        shared_robot_state.q[j]   = state.q[j];
                        shared_robot_state.q_p[j] = state.dq[j];
                        // finite difference for q_pp
                        shared_robot_state.q_pp[j] = state.ddq_d[j];
                    }
                }

                // Read measured state
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_m(state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_m(state.dq.data());

                q_meas = q_m;

                // ---- T-TIME / R-IDLE bookkeeping (no locking: this thread is the
                // sole writer of g_t_log/g_idle_time while the playback loop runs) ----
                {
                    double dt_step = period_s;
                    g_t_log += dt_step;
                }

                // stop condition check
                // stop event tolerance check
                Eigen::Index max_idx;
                double max_diff = (q_end - q_meas).cwiseAbs().maxCoeff(&max_idx);
                bool do_stop = max_diff > TOLL_NO_STOP;
                //std::cout << "max distance: " << max_diff << " at index " << max_idx << "\n";

                if (current_state == RUNNING && stop_flag.load() && do_stop) {
                    g_stop_count.fetch_add(1);  // R-STOPS: a genuine safety stop is triggered
                    stop_flag_local = stop_flag.load();
                    
                    stop_duration = shared_stop_duration.load();

                    // Index of the nominal sample that is being played: the
                    // playback resumes from it once the obstacle has cleared.
                    idx_stop = idx_start + static_cast<size_t>(
                        std::min(static_cast<int>(t / TRAJ_STEP), num_points - 1));

                    q_0_brake = q_m;
                    v_0_brake = dq_m;
                    a_0_brake = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.ddq_d.data());

                    stop_num_points = static_cast<int>(stop_duration / TRAJ_STEP) + 1;
                    // compute poly5 for stopping
                    Q_stop.resize(stop_num_points, 7);
                    Q_p_stop.resize(stop_num_points, 7);
                    Q_pp_stop.resize(stop_num_points, 7);
                    for (int j = 0; j < 7; ++j) {
                        Eigen::MatrixXd tj = trajPoly5Numeric(
                            q_0_brake(j), v_0_brake(j), a_0_brake(j),
                            q_0_brake(j), 0.0, 0.0,
                            stop_duration, stop_num_points);
                        Q_stop.col(j) = tj.col(0);
                        Q_p_stop.col(j) = tj.col(1);
                        Q_pp_stop.col(j) = tj.col(2);
                    }

                    t_brake = 0.0;
                    current_state = STOPPING;
                }

                // Desired position & velocity placeholders
                Eigen::Matrix<double, 7, 1> q_d;
                Eigen::Matrix<double, 7, 1> dq_d;
                Eigen::Matrix<double, 7, 1> ddq_d;
                Eigen::Matrix<double, 7, 1> tau_d = Eigen::Matrix<double, 7, 1>::Zero();

                // compute target values
                if (current_state == STOPPING) {
                    t_brake += period_s;
                    if (t_brake >= stop_duration) {
                        {
                            std::lock_guard<std::mutex> lock(robot_state_mutex);
                            std::fill(shared_robot_state.q_p.begin(), shared_robot_state.q_p.end(), 0.0);
                            std::fill(shared_robot_state.q_pp.begin(), shared_robot_state.q_pp.end(), 0.0);
                        }
                        // motion finished (the old code ended the motion here
                        // with zero torques)
                        motion_finished = true;
                    } else {
                        int stop_idx = static_cast<int>(t_brake / TRAJ_STEP);
                        stop_idx = std::min(stop_idx, stop_num_points - 1);

                        q_d = Q_stop.row(stop_idx).transpose();
                        dq_d = Q_p_stop.row(stop_idx).transpose();
                        ddq_d = Q_pp_stop.row(stop_idx).transpose();
                    }
                } 
                else if (current_state == RUNNING) {
                    // Normal execution
                    t += period_s;
                    //  Get current trajectory index 
                    int idx = static_cast<int>(t / TRAJ_STEP);
                    idx = std::min(idx, num_points - 1);

                    //  Check if trajectory is complete 
                    if (t >= traj_duration_new) {
                        motion_finished = true;
                    } else {
                        q_d = Q.row(idx).transpose();
                        dq_d = Q_p.row(idx).transpose();
                        ddq_d = Q_pp.row(idx).transpose();
                    }
                }

                if (!motion_finished) {
                    // Fetch dynamic properties from the Pinocchio model
                    // (replaces franka::Model: only the Coriolis term is used,
                    // as feed-forward of the joint impedance law)
                    Eigen::VectorXd q_v   = Eigen::Map<const Eigen::VectorXd>(state.q.data(),   7);
                    Eigen::VectorXd dq_v  = Eigen::Map<const Eigen::VectorXd>(state.dq.data(),  7);
                    pinocchio::computeCoriolisMatrix(robot.getModel(), data, q_v, dq_v);
                    Eigen::VectorXd coriolis = data.C * dq_v;

                    // compute torques
                    Eigen::Matrix<double, 7, 1> tau_fb;
                    Eigen::Matrix<double, 7, 1> tau_ff = /*mass * ddq_d +*/ coriolis;

                    // PD controller
                    for (size_t i = 0; i < 7; ++i) {
                        tau_fb(i) = k_gains[i]*(q_d(i)-q_m(i)) + 
                                    d_gains[i]*(dq_d(i)-dq_m(i));
                    }

                    tau_d = tau_ff + tau_fb;
                }

                std::array<double, 7> tau_cmd;
                Eigen::VectorXd::Map(&tau_cmd[0], 7) = tau_d;
                std::array<double, 7> tau_d_rate_limited = limitTorqueRate(tau_cmd, state.tau_J_d);

                if (!motion_finished) {
                    // End-effector position from Pinocchio FK (replaces
                    // state.O_T_EE from libfranka)
                    pinocchio::forwardKinematics(robot.getModel(), data, q_meas);
                    pinocchio::updateFramePlacements(robot.getModel(), data);
                    const Eigen::Vector3d ee_pos =
                        data.oMf[EE_ID].translation();

                    LogSample s;
                    s.t = g_t_log;
                    for (int j = 0; j < 7; ++j) {
                        s.q[j]          = state.q[j];
                        s.dq[j]         = state.dq[j];
                        s.tau_cmd[j]    = tau_d_rate_limited[j];
                        s.tau_meas[j]   = state.tau_J[j];
                    }
                    s.ee_pos = {ee_pos.x(), ee_pos.y(), ee_pos.z()};
                    log_queue.push(s);

                    // Update the simulated plant: with no robot attached the
                    // "measured" state follows the commanded trajectory
                    for (int j = 0; j < 7; ++j) {
                        state.q[j]      = q_d(j);
                        state.dq[j]     = dq_d(j);
                        state.ddq_d[j]  = ddq_d(j);
                    }
                    state.tau_J_d = tau_d_rate_limited;
                    state.tau_J   = tau_d_rate_limited;
                }

                // Publish the commanded trajectory sample to web_interface.py
                // (this replaces streaming the sample to the real robot)
                t_publish += period_s;
                if (t_publish >= TRAJ_PUBLISH_PERIOD) {
                    robot.ComputeFK(q_meas);
                    publisher.publishSample(robot.getModel(), robot.getData(), q_meas);
                    t_publish = 0.0;
                }

                if (motion_finished) break;

                // Pace the playback at the old 1 kHz control rate
                next_tick += std::chrono::milliseconds(1);
                std::this_thread::sleep_until(next_tick);
            }
        } catch (const std::exception& e) {
                std::cerr << "executeTrajectory: playback fault during main motion: " << e.what() << "\n";
                throw;
        }

        if (stop_flag_local) {
            std::cout << "\t Traiettoria di stop completata\n";

            // The robot braked to a full stop where the safety stop was
            // triggered: the nominal trajectory is resumed from the sample that
            // was being played, so that the motion continues with the very same
            // shape and timing as the recorded one.
            q_c = q_meas;          // restart from where we stopped
            idx_start = idx_stop;  // and from the sample that was being played

            // wait for obstacle to move -- this is real idle time: the robot is
            // fully stopped, so no real-time deadline applies here and we can
            // just measure wall-clock duration directly.
            // (Same-thread access to g_idle_time/g_t_log, so still no locking needed.)
            // The wait is bounded: if the human never leaves the workspace (or
            // the skeleton stream is dead, so the distance stays NaN) the
            // robot resumes after MAX_STOP_WAIT_S instead of waiting forever.
            auto pause_start = std::chrono::steady_clock::now();
            while (true) {
                double wait_elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - pause_start).count();
                if (wait_elapsed >= MAX_STOP_WAIT_S) {
                    std::cerr << "executeTrajectory: obstacle still blocking after "
                              << MAX_STOP_WAIT_S << "s, resuming anyway\n";
                    break;
                }
                if (shared_rh_distance.load() >= 0.5) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            double pause_duration = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - pause_start).count();
            g_idle_time += pause_duration;  // whole pause counts as idle, no debounce needed
            g_t_log     += pause_duration;  // T-TIME must include the pause, not just loop time

            // Reset stop flag for retry
            stop_flag.store(false);
            current_state = RUNNING;
        } else
        {
            //  Robot reached target 
            flag_continue = false;
            std::cout << "Fine del movimento alla configurazione finale\n";
        }
    }
}


int execute_task (int n_traj, const std::filesystem::path& root) {
    // Caricamento del modello URDF del robot Panda utilizzando Pinocchio per calcoli cinematici avanzati
    const std::filesystem::path urdf_path = root / "src" / "urdf" / "panda.urdf";
    if (!std::filesystem::exists(urdf_path)) {
        std::cerr << "URDF not found: " << urdf_path << "\n";
        return 1;
    }
    RobotModel robot(urdf_path.string());

    // ── Nominal trajectory ───────────────────────────────────────────────────
    // The set under src/trajectories/test<N>/ holds the waypoints (p_ref.csv,
    // Cartesian, or q_ref.csv, joint space) and the cumulative time at which
    // each of them is reached (t_ref.csv). The whole motion is assembled here,
    // already sampled at the control rate: the playback just replays these
    // samples.
    const std::filesystem::path traj_dir =
        root / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    // Number of samples reserved for each segment of the nominal trajectory
    // (a reserve hint for the loader: the real length of a segment follows
    // from its duration and from the control rate).
    constexpr int NOMINAL_SAMPLES = 4000;

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

    // ── Configurazione di partenza ─────────────────────────────────────────
    // The nominal trajectory starts at the first recorded waypoint, so the
    // robot is moved there before the playback begins.
    const Eigen::VectorXd q_task_start = nominal_traj.q.front();

    // movimento del robot
    std::cout << "This will move the robot to the initial joint configuration!\n";
    try {
        // The trajectory publisher replaces the libfranka robot handle: the
        // samples that used to be streamed to the robot at 1 kHz are now
        // published to web_interface.py via DataTransmitter.
        TrajectoryPublisher publisher;

        std::array<double, 7> q_start;
        {
            // The simulated robot starts from the home configuration, which is
            // also what the collision checker sees before the first control tick
            std::lock_guard<std::mutex> lock(robot_state_mutex);
            for (int j = 0; j < 7; ++j) {
                shared_robot_state.q[j] = Q_HOME[j];
                q_start[j] = Q_HOME[j];
            }
        }

        // Duration of the move to the initial joint configuration: it scales
        // with the distance to travel and is capped, so that a configuration
        // that is already the starting one does not delay the task.
        const Eigen::VectorXd q_start_eig =
            Eigen::Map<const Eigen::Matrix<double, 7, 1>>(q_start.data());
        const double max_joint_dist = (q_task_start - q_start_eig).cwiseAbs().maxCoeff();
        const double move_duration = std::clamp(20.0 * max_joint_dist, 0.5, 5.0);

        pinocchio::Data& move_data = robot.getData();
        auto next_tick = std::chrono::steady_clock::now();

        // Simulated playback of the move to the initial joint configuration:
        // the same smoothstep interpolation as before, at the old 1 kHz rate
        for (int tick = 0; ; ++tick) {
            // period 0 on the first cycle, then the fixed 1 kHz step
            const double time = tick * TRAJ_STEP;
            if (time > move_duration) break;

            // Smooth step interpolation (zero velocity at start and end)
            double s = time / move_duration;
            s = std::min(s, 1.0);
            double alpha = s * s * (3.0 - 2.0 * s);  // smoothstep

            Eigen::VectorXd q_cmd(7);
            for (int j = 0; j < 7; ++j)
                q_cmd[j] = q_start[j] + alpha * (q_task_start(j) - q_start[j]);

            // stream the commanded configuration to web_interface.py
            robot.ComputeFK(q_cmd);
            publisher.publishSample(robot.getModel(), robot.getData(), q_cmd);

            // update the shared state the collision checker reads
            {
                std::lock_guard<std::mutex> lock(robot_state_mutex);
                for (int j = 0; j < 7; ++j) {
                    shared_robot_state.q[j] = q_cmd(j);
                    shared_robot_state.q_p[j] = 0.0;
                    shared_robot_state.q_pp[j] = 0.0;
                }
            }

            next_tick += std::chrono::milliseconds(1);
            std::this_thread::sleep_until(next_tick);
        }

        std::cout << "Finished moving to initial joint configuration\n";
        std::cout << "WARNING: Collision thresholds are set to high values.\n";
        std::cout << "Make sure you have the user stop at hand!\n";
        std::cout << "Starting the joint control task\n";

        
        


        // apri file per logging
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
        std::ofstream outfile_q(pathname + "q_robot.xml");
        std::ofstream outfile_dist(pathname + "dist_segment.xml");
        std::ofstream outfile_dq(pathname + "dq_robot.xml");
        std::ofstream outfile_tau(pathname + "tau_robot.xml");
        std::ofstream outfile_pose(pathname + "pose_robot.xml");

        outfile_q << "<q>\n";
        outfile_dist << "<distance>\n";
        outfile_dq << "<dq>\n";
        outfile_tau << "<tau>\n";
        outfile_pose << "<pose>\n";
        std::cout << "Outfiles initiated\n";

        // thread per scheletro e collision check
        // il processo live riceve lo scheletro fuso pubblicato da data_merging.py
        // (topic "MERGED", device_id 10 -> porta 6001 + 10 = 6011)
        std::thread skeleton_thread(receiveSkeletonMerged, "localhost", 10, "MERGED",
                                    pathname + "skeleton_received.txt");

        std::thread collision_thread(collisionCheckerThread,
        std::ref(robot), std::ref(outfile_dist), std::ref(publisher));
        
        std::thread logger_thread(loggerThread, std::ref(outfile_q), std::ref(outfile_dq), std::ref(outfile_tau), std::ref(outfile_pose));

        //StopDurationOptimizer optimizer(model);

        auto task_start = std::chrono::steady_clock::now();  // real wall-clock, diagnostic only

        // esegui la traiettoria nominale
        try {
            executeTrajectory(nominal_traj, publisher, robot);
        } catch (...) {
            // A playback fault propagated up from executeTrajectory. Still need to
            // stop the skeleton/collision threads and close the log files
            // properly before this task ends otherwise a fault leaks
            // threads and leaves truncated/unflushed log files behind, on top
            // of the actual error.
            std::cerr << "executeTask: aborting due to playback fault, cleaning up\n";
            trajectory_done.store(true);
            skeleton_thread.join();
            collision_thread.join();
            logger_thread.join();
            outfile_q << "</q>\n";
            outfile_dist << "</distance>\n";
            outfile_dq << "</dq>\n";
            outfile_tau << "</tau>\n";
            outfile_pose << "</pose>\n";
            outfile_q.close();
            outfile_dist.close();
            outfile_dq.close();
            outfile_tau.close();
            outfile_pose.close();
            flushIdleStreak();
            writeMetricsFile(pathname);  // still save whatever metrics were gathered before the fault
            throw;  // let the caller (main) know the task did not complete

        }

        double real_elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - task_start).count();
        const double nominal_duration = nominal_traj.t.empty() ? 0.0 : nominal_traj.t.back();
        std::cout << "Nominal trajectory duration: " << nominal_duration
                << "s, real wall-clock elapsed: " << real_elapsed << "s\n";

        trajectory_done.store(true);

        skeleton_thread.join();
        collision_thread.join();
        logger_thread.join();

        outfile_q << "</q>\n";
        outfile_dist << "</distance>\n";
        outfile_dq << "</dq>\n";
        outfile_tau << "</tau>\n";
        outfile_pose << "</pose>\n";
        outfile_q.close();
        outfile_dist.close();
        outfile_dq.close();
        outfile_tau.close();
        outfile_pose.close();

        flushIdleStreak();
        writeMetricsFile(pathname);

        std::cout << "Trajectory completed, " << nominal_traj.t.back()
                << " seconds have passed, shutting down example\n";

    } catch (const std::exception& ex) {
        // Anything unexpected still exits cleanly with a clear message
        // rather than letting it terminate the process unhandled.
        std::cerr << "Unexpected error: " << ex.what() << "\n";
        trajectory_done.store(true);
        return -1;
    }

    // The task completed normally: return an explicit success code. Falling off
    // the end of a non-void function is undefined behaviour and, with the
    // optimisations enabled here, it made the process jump back into the
    // already-destroyed publisher scope once the run was over.
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Locate the project root, i.e. the directory holding src/urdf/,
// src/trajectories/ and logs/.
//
// The root is looked up in this order:
//   1) the path given on the command line (run.sh passes the repository root)
//   2) the current working directory
//   3) the parent of the executable's directory (build/ -> project root)
//
// so that the demo can be launched either from the build directory or from the
// project root, with no path depending on the current working directory.
// ─────────────────────────────────────────────────────────────────────────────
static std::filesystem::path findProjectRoot(const std::string& explicit_root = "") {
    namespace fs = std::filesystem;
    std::error_code ec;

    const auto is_root = [](const fs::path& p) {
        std::error_code ec2;
        return fs::exists(p / "src" / "urdf" / "panda.urdf", ec2);
    };

    // 1) the path given on the command line
    if (!explicit_root.empty()) {
        fs::path p = fs::path(explicit_root);
        if (p.is_relative()) p = fs::absolute(p, ec);
        if (!ec && is_root(p)) return p;
    }

    // 2) the current working directory
    fs::path cwd = fs::current_path(ec);
    if (!ec && is_root(cwd)) return cwd;

    // 3) the parent of the executable's directory (build/ -> project root)
    fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        fs::path root = exe.parent_path().parent_path();
        if (is_root(root)) return root;

        root = exe.parent_path();
        if (is_root(root)) return root;
    }

    return cwd;  // fallback: the files below will simply fail to open
}


int main(int argc, char** argv) {
    // ── Command line: <trajectory index> [repository root] ──────────────────
    // The trajectory index selects the set under src/trajectories (test<N>/),
    // the repository root is the directory holding src/ and scripts/.
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
