/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░█░▀█▀░▀█▀░█░░░█▀▀
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░█░░█░░░█░░█░░░▀▀█
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀▀▀░▀▀▀

trajectory_utils.cpp - Quintic trajectory generation and point-to-point trajectory assembly with IK on the recorded waypoints.
*/

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "trajectory_utils.hpp"
#include "robot_model.hpp"
#include "trajectory_loader.hpp"
#include "core.hpp"

namespace fs = std::filesystem;
std::vector<Eigen::VectorXd> q_robot;
std::vector<double> t_robot;
std::vector<double> t_data;

// ─────────────────────────────────────────────────────────────────────────────
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n,
                     const Eigen::VectorXd& v0,
                     const Eigen::VectorXd& a0,
                     const Eigen::VectorXd& v1,
                     const Eigen::VectorXd& a1) {
    const int d = static_cast<int>(x0.size());
    assert(x1.size() == d && v0.size() == d && a0.size() == d &&
           v1.size() == d && a1.size() == d);

    Traj tr{Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n)};

    const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

    const Eigen::VectorXd c0 = x0;
    const Eigen::VectorXd c1 = v0;
    const Eigen::VectorXd c2 = 0.5 * a0;

    const Eigen::VectorXd c3 = ( 20.0 * (x1 - x0) - (8.0 * v1 + 12.0 * v0) * T
                                 - (3.0 * a0 - a1) * T2) / (2.0 * T3);
    const Eigen::VectorXd c4 = (-30.0 * (x1 - x0) + (14.0 * v1 + 16.0 * v0) * T
                                 + (3.0 * a0 - 2.0 * a1) * T2) / (2.0 * T4);
    const Eigen::VectorXd c5 = ( 12.0 * (x1 - x0) - 6.0 * (v1 + v0) * T
                                 - (a0 - a1) * T2) / (2.0 * T5);

    for (int k = 0; k < n; ++k) {
        const double t  = std::min(k * dt, T);
        const double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;

        tr.p.col(k) = c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
        tr.v.col(k) = c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4;
        tr.a.col(k) = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3;
    }
    return tr;
}


// ─────────────────────────────────────────────────────────────────────────────
// Append one quintic segment to the output trajectory
// ─────────────────────────────────────────────────────────────────────────────
void build_p2p_traj(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                    std::vector<double>* vnorm, Trajectory& out, const double t_end,
                    double& t, RobotModel& robot, double dt,
                    const Eigen::VectorXd& va,
                    const Eigen::VectorXd& aa,
                    const Eigen::VectorXd& vb,
                    const Eigen::VectorXd& ab) {
    // The segment is sampled at the control rate, so that the playback
    // takes exactly the recorded duration.

    const int n = std::max(2, static_cast<int>(std::lround(t_end / dt)));

    Traj tr = QuinticPolyTraj(a, b, t_end, dt, n, va, aa, vb, ab);

    for (int k = 0; k < n; ++k) {
        const Eigen::VectorXd q   = tr.p.col(k);
        const Eigen::VectorXd qd  = tr.v.col(k);
        const Eigen::VectorXd qdd = tr.a.col(k);

        const Eigen::Matrix<double, 6, 1> v = robot.ComputeJacobian("panda_link8", q) * qd;

        Waypoint wp;
        wp.q = q;
        wp.qd = qd;
        wp.qdd = qdd;
        wp.p = robot.GetJointPose("panda_link8", q).translation();
        wp.pd = v.head<3>();
        wp.timestamp = dt * t++;
        out.wp.push_back(wp);
        if (vnorm) vnorm->push_back(v.norm());
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Compute added time for restarting the trajectory
// ─────────────────────────────────────────────────────────────────────────────
double computeAddedTime(const Eigen::VectorXd& q_end, const Eigen::VectorXd& q_c)
{
    static const KinematicsLimits limits;

    // Upper limits (row 1) as column vectors
    const Eigen::VectorXd q_p_lim  = limits.qd_limits.row(1).transpose();
    const Eigen::VectorXd q_pp_lim = limits.qdd_limits.row(1).transpose();

    const Eigen::VectorXd dq = q_end - q_c;

    // Velocity-limited time
    const Eigen::VectorXd vect_t_vel = 5*(15.0 * dq).cwiseQuotient(8.0 * q_p_lim);
    const double t_vel = vect_t_vel.cwiseAbs().maxCoeff();

    // Acceleration-limited time
    const Eigen::VectorXd vect_t_acc = 5*(10.0 * dq).cwiseQuotient(std::sqrt(3.0) * q_pp_lim);
    const double t_acc = vect_t_acc.cwiseAbs().cwiseSqrt().maxCoeff();

    // Enforce minimum duration of 0.5 s
    return std::max({t_vel, t_acc, 0.5});
}


// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj,
                               double dt) {

    // ── Waypoint and time loading ───────────────────────────────────────────
    const fs::path traj_dir =
        fs::path(c_dir) / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    // The waypoint format (Cartesian or joint space) is detected from the
    // files that are present in the trajectory directory.
    TrajectoryData data = loadTrajectorySet(traj_dir);

    if (data.p.empty() && data.q.empty()) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory holds no waypoints");
    }
    if (data.t.size() < 2) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory needs at least 2 waypoints");
    }

    // ── Joint waypoints ─────────────────────────────────────────────────────
    // Cartesian waypoints are solved with the inverse kinematics, joint
    // waypoints are already configurations and are used as they are.
    Eigen::VectorXd q_base(7);
    q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

    // Fixed orientation of the end effector for the Cartesian waypoints (only
    // the position is taken from the file).
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(1, 1) = -1.0;
    R(2, 2) = -1.0;

    if (data.isJointSpace()) {
        for (auto q : data.q) {
            q_robot.push_back(q);
        }
        std::cout << "Loaded " << q_robot.size() << " joint-space waypoints from "
                  << traj_dir << std::endl;
    }
    else {
        q_robot.reserve(data.p.size());
        for (size_t i = 0; i < data.p.size(); ++i) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = R;
            T.translation() = data.p[i];

            Eigen::VectorXd q;
            if (!robot.ComputeIK("panda_link8", T, q_base, &q)) {
                // The IK did not converge from the reference configuration: try
                // again starting from the solution of the previous waypoint,
                // which is usually much closer to the target.
                bool ok = false;
                if (i > 0) {
                    ok = robot.ComputeIK("panda_link8", T, q_robot.back(), &q);
                }
                if (!ok) {
                    throw std::runtime_error("IK failed for waypoint " + std::to_string(i) +
                                             " of " + traj_dir.string());
                }
            }
            q_robot.push_back(q);
        }
        std::cout << "Loaded " << q_robot.size() << " Cartesian waypoints from "
                  << traj_dir << " and solved the IK for each of them" << std::endl;
    }

    for (auto t : data.t) {
        t_robot.push_back(t);
    }

    t_data = t_robot;

    // ── Segment assembly ────────────────────────────────────────────────────
    const size_t n_seg = q_robot.size() - 1;
    std::vector<double> v_norm;
    int N = t_robot[t_robot.size() - 1]/dt;
    Trajectory traj;
    // traj.status.push_back("running");
    traj.wp.reserve(N);

    double t = 0;
    for (size_t i = 0; i < n_seg; ++i) {
        const double t_end = t_robot[i + 1] - t_robot[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        build_p2p_traj(q_robot[i], q_robot[i + 1], &v_norm, traj, t_end, t, robot, dt);
    }

    if (traj.wp.empty()) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory holds no motion");
    }

    // ── Velocity statistics ─────────────────────────────────────────────────
    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());

    return traj;
}


// ─────────────────────────────────────────────────────────────────────────────
// Rebuild helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Number of samples build_p2p_traj produces for the segments [0, n_seg)
size_t sampled_prefix(const std::vector<double>& t_wp, size_t n_seg, double dt) {
    size_t n = 0;
    for (size_t i = 0; i < n_seg && i + 1 < t_wp.size(); ++i) {
        const double t_end = t_wp[i + 1] - t_wp[i];
        if (t_end <= 0.0) continue;
        n += static_cast<size_t>(std::max(2, static_cast<int>(std::lround(t_end / dt))));
    }
    return n;
}

// Drop every sample that belongs to a segment which is going to be rebuilt and
// return the number of samples that are kept.
size_t truncate_trajectory(Trajectory& traj, size_t n_keep) {
    n_keep = std::min(n_keep, traj.wp.size());
    traj.wp.resize(n_keep);
    return n_keep;
}

}  // namespace


// ─────────────────────────────────────────────────────────────────────────────
// Rebuild a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
void old_restart_trajectory(Trajectory& traj, RobotModel& robot,
                        const Eigen::VectorXd& q_restart,
                        const double& t_restart, double dt) {
    // The restart configuration usually references an element of traj
    // (traj.wp[r_idx].q), which is modified below: work on a copy.
    const Eigen::VectorXd q_restart_c = q_restart;

    // ── Restart time ────────────────────────────────────────────────────────
    // Only the waypoints whose timestamp is strictly smaller than the restart
    // time are kept: the trajectory is rebuilt up to that point.
    if (!(t_restart > 0.0)) return;

    size_t ins = 0;
    while (ins < t_robot.size() && t_robot[ins] < t_restart) ins++;

    std::vector<Eigen::VectorXd> q_robot_temp;
    std::vector<double> t_robot_temp;
    q_robot_temp.reserve(q_robot.size() + 1);
    t_robot_temp.reserve(q_robot.size() + 1);

    // std::cout << std::endl;
    // std::cout << "t_data: ";
    // for (const auto& t : t_data) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // Offset added to the recorded timestamps of the waypoints that are
    // still to be reached. It is computed from the time actually left in
    // the interrupted segment once the restart point is inserted.
    double t_gap = 0.0;
    for (size_t i = 0; i <= q_robot.size(); ++i) {
        double t_ref = t_data[t_data.size() - (q_robot.size() - i)];
        if (i == ins) {
            t_gap = t_restart + computeAddedTime(q_robot[i], q_restart_c) - t_ref;
        }

        if (i == q_robot.size()) break;
        i < ins ? t_robot_temp.push_back(t_robot[i]) : t_robot_temp.push_back(t_ref + t_gap);
        q_robot_temp.push_back(q_robot[i]);
    }

    q_robot = q_robot_temp;
    t_robot = t_robot_temp;

    // std::cout << std::endl;
    // std::cout << "t_robot: ";
    // for (const auto& t : t_robot) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // ── Partial segment assembly ────────────────────────────────────────────
    // The waypoints before the restart point are unchanged, so the samples of
    // the trajectory leading up to them are kept. Only the segment that ends
    // at the newly inserted waypoint (ins - 1) and the following ones are
    // rebuilt, starting from the insertion index.
    const size_t first_seg = ins > 0 ? ins - 1 : 0;
    const size_t keep = truncate_trajectory(traj, sampled_prefix(t_robot, first_seg, dt));
    double t = dt * static_cast<double>(keep);

    for (size_t i = first_seg; i + 1 < q_robot.size(); ++i) {
        const double t_end = t_robot[i + 1] - t_robot[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt);
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Rebuild a point-to-point trajectory resuming from current motion status
// ─────────────────────────────────────────────────────────────────────────────
// Same structure as restart_trajectory, but the first rebuilt segment starts
// from the current velocity and acceleration (qd_resume, qdd_resume) instead of
// a steady status (zero velocity/acceleration), so the robot keeps its motion
// momentum when resuming the remaining trajectory.
// ─────────────────────────────────────────────────────────────────────────────
void restart_trajectory(Trajectory& traj, RobotModel& robot,
                       const Eigen::VectorXd& q_resume,
                       const Eigen::VectorXd& qd_resume, const Eigen::VectorXd& qdd_resume,
                       const double& t_resume, double dt) {
    // The resume status usually references an element of traj
    // (traj.wp[r_idx].q), which is modified below: work on copies.
    const Eigen::VectorXd q_resume_c  = q_resume;
    const Eigen::VectorXd qd_resume_c = qd_resume;
    const Eigen::VectorXd qdd_resume_c = qdd_resume;

    // std::cout << "q_resume_c = " << q_resume_c.transpose() << std::endl;

    // ── Resume time ────────────────────────────────────────────────────────
    // Only the waypoints whose timestamp is strictly smaller than the resume
    // time are kept: the trajectory is rebuilt from that point onward.
    if (!(t_resume > 0.0)) return;

    size_t ins = 0;
    while (ins < t_robot.size() && t_robot[ins] < t_resume) ins++;

    std::vector<Eigen::VectorXd> q_robot_temp;
    std::vector<double> t_robot_temp;
    q_robot_temp.reserve(q_robot.size() + 1);
    t_robot_temp.reserve(q_robot.size() + 1);


    // std::cout << "q_robot (size " << q_robot.size() << "):\n";
    // for (size_t i = 0; i < q_robot.size(); ++i) {
    //     std::cout << "  [" << i << "] " << q_robot[i].transpose() << "\n";
    // }
// 
    // std::cout << "t_robot (size " << t_robot.size() << "):\n";
    // for (size_t i = 0; i < t_robot.size(); ++i) {
    //     std::cout << "  [" << i << "] " << t_robot[i] << "\n";
    // }




    // Offset added to the recorded timestamps of the waypoints that are
    // still to be reached. It is computed from the time actually left in
    // the interrupted segment once the resume point is inserted.
    double t_gap = 0.0;
    for (size_t i = 0; i <= q_robot.size(); ++i) {
        double t_ref = t_data[t_data.size() - (q_robot.size() - i)];
        if (i == ins) {
            t_gap = t_resume + computeAddedTime(q_robot[i], q_resume_c) - t_ref;
            q_robot_temp.push_back(q_resume_c);
            t_robot_temp.push_back(t_resume);
        }

        if (i == q_robot.size()) break;
        i < ins ? t_robot_temp.push_back(t_robot[i]) : t_robot_temp.push_back(t_ref + t_gap);
        q_robot_temp.push_back(q_robot[i]);
    }

    

    q_robot = q_robot_temp;
    t_robot = t_robot_temp;




    // std::cout << "q_robot (size " << q_robot.size() << "):\n";
    // for (size_t i = 0; i < q_robot.size(); ++i) {
    //     std::cout << "  [" << i << "] " << q_robot[i].transpose() << "\n";
    // }
// 
    // std::cout << "t_robot (size " << t_robot.size() << "):\n";
    // for (size_t i = 0; i < t_robot.size(); ++i) {
    //     std::cout << "  [" << i << "] " << t_robot[i] << "\n";
    // }




    // ── Partial segment assembly ────────────────────────────────────────────
    // The waypoints before the resume point are unchanged, so the samples of
    // the trajectory leading up to them are kept. Only the segment that ends
    // at the resume point (ins - 1) and the following ones are rebuilt, starting
    // from the insertion index. The first rebuilt segment starts from the
    // current velocity and acceleration instead of a steady status.
    const size_t first_seg = ins > 0 ? ins - 1 : 0;
    const size_t keep = truncate_trajectory(traj, sampled_prefix(t_robot, first_seg, dt));
    double t = dt * static_cast<double>(keep);

    for (size_t i = first_seg; i + 1 < q_robot.size(); ++i) {
        const double t_end = t_robot[i + 1] - t_robot[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt);
        // if (i == first_seg) {
        //     // Resume from current velocity/acceleration instead of steady status.
        //     build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt,
        //                    qd_resume_c, qdd_resume_c);
        // }
        // else {
        //     build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt);
        // }
    }
}


// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point emergency stop trajectory
// ─────────────────────────────────────────────────────────────────────────────
size_t stop_trajectory(Trajectory& traj, RobotModel& robot,
                       const Eigen::VectorXd& q_stop,
                       const Eigen::VectorXd& qd_stop, const Eigen::VectorXd& qdd_stop,
                       const double& t_stop, const double& stop_duration, double dt) {
    // The stop status usually references an element of traj (traj.wp[r_idx].q),
    // which is modified below: work on copies.
    const Eigen::VectorXd q_stop_c  = q_stop;
    const Eigen::VectorXd qd_stop_c = qd_stop;
    const Eigen::VectorXd qdd_stop_c = qdd_stop;

    // ── Adding stop waypoint ────────────────────────────────────────────────
    size_t ins = 0;
    while (ins < t_robot.size() && t_robot[ins] < t_stop) ins++;

    std::vector<Eigen::VectorXd> q_robot_temp;
    std::vector<double> t_robot_temp;
    q_robot_temp.reserve(q_robot.size() + 2);
    t_robot_temp.reserve(q_robot.size() + 2);

    // std::cout << std::endl;
    // std::cout << "t_data: ";
    // for (const auto& t : t_data) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // Offset added to the recorded timestamps of the waypoints that are
    // still to be reached. It is computed from the time actually left in
    // the interrupted segment once the restart point is inserted.
    for (size_t i = 0; i <= q_robot.size(); ++i) {
        double t_ref = t_data[t_data.size() - (q_robot.size() - i)];
        if (i == ins) {
            q_robot_temp.push_back(q_stop_c);
            q_robot_temp.push_back(q_stop_c);
            t_robot_temp.push_back(t_stop);
            t_robot_temp.push_back(t_stop + stop_duration);
        }

        if (i == q_robot.size()) break;
        i < ins ? t_robot_temp.push_back(t_robot[i]) : t_robot_temp.push_back(t_robot[i] + stop_duration);
        q_robot_temp.push_back(q_robot[i]);
    }

    q_robot = q_robot_temp;
    t_robot = t_robot_temp;  

    // std::cout << std::endl;
    // std::cout << "t_robot: ";
    // for (const auto& t : t_robot) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;


    // ── Partial segment assembly ────────────────────────────────────────────
    // The waypoints before the stop point are unchanged, so the samples of the
    // trajectory leading up to them are kept. Only the segment that ends at
    // the inserted stop waypoint (ins - 1) and the following ones are rebuilt,
    // starting from the insertion index.
    const size_t first_seg = ins > 0 ? ins - 1 : 0;
    const size_t keep = truncate_trajectory(traj, sampled_prefix(t_robot, first_seg, dt));
    double t = dt * static_cast<double>(keep);

    for (size_t i = first_seg; i + 1 < q_robot.size(); ++i) {
        const double t_end = t_robot[i + 1] - t_robot[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        if (i == ins) {
            build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt,
                           qd_stop_c, qdd_stop_c);
        }
        else {
            build_p2p_traj(q_robot[i], q_robot[i + 1], nullptr, traj, t_end, t, robot, dt);
        }
    }

    // Index of the last sample of the inserted stop: the stop segment is at
    // index `ins` (q_stop -> q_stop) and spans the hold duration.
    const size_t stop_end = sampled_prefix(t_robot, ins + 1, dt);
    return stop_end > 0 ? stop_end - 1 : 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory with a trapezoidal velocity profile.
// ─────────────────────────────────────────────────────────────────────────────
// Starts from a steady-status waypoint (zero velocity/acceleration), accelerates
// to `velocity_PFL`, cruises at constant PFL velocity, and finally decelerates
// to reach the following waypoint with zero velocity and acceleration. Each
// phase is a quintic segment chained so the velocity is continuous while the
// acceleration steps at the phase transitions (inherent to a trapezoidal
// profile). Samples are appended to `traj` starting at `t_start`.
// ─────────────────────────────────────────────────────────────────────────────
void limit_velocity(Trajectory& traj, RobotModel& robot,
                    const Eigen::VectorXd& q_start,
                    const Eigen::VectorXd& q_end,
                    const double& t_start, double dt,
                    double velocity_PFL) {
    // Kinematic limits (same source as computeAddedTime / restart_trajectory).
    static const KinematicsLimits limits;
    const Eigen::VectorXd a_max_lim = limits.qdd_limits.row(1).transpose();

    const Eigen::VectorXd q_start_c = q_start;
    const Eigen::VectorXd q_end_c = q_end;

    // Path from q_start to q_end; path coordinate s in [0, d_max].
    const Eigen::VectorXd D = q_end_c - q_start_c;
    const double d_max = D.cwiseAbs().maxCoeff();
    if (d_max <= 1e-12) {
        // No motion: just append the end waypoint.
        Waypoint wp;
        wp.q = q_end_c;
        wp.qd = Eigen::VectorXd::Zero(7);
        wp.qdd = Eigen::VectorXd::Zero(7);
        wp.p = robot.GetJointPose("panda_link8", q_end_c).translation();
        wp.pd = Eigen::Vector3d::Zero();
        wp.timestamp = t_start + dt;
        traj.wp.push_back(wp);
        return;
    }
    const Eigen::VectorXd u = D / d_max;  // unit direction along the path

    // Common path acceleration: limited by the most constrained joint.
    double a_path = a_max_lim.minCoeff();
    if (a_path <= 1e-9) a_path = 1.0;

    const double v_path = std::max(velocity_PFL, 1e-9);

    // Trapezoidal velocity profile along the path coordinate.
    const double d_acc = v_path * v_path / (2.0 * a_path);
    double t_acc = 0.0, t_cruise = 0.0, t_dec = 0.0;
    double s1 = 0.0, s2 = 0.0;
    Eigen::VectorXd v_cruise = Eigen::VectorXd::Zero(7);

    if (2.0 * d_acc < d_max) {
        // Full trapezoid: accelerate, cruise at v_path, decelerate.
        t_acc = v_path / a_path;
        t_dec = v_path / a_path;
        t_cruise = (d_max - 2.0 * d_acc) / v_path;
        s1 = d_acc;
        s2 = d_max - d_acc;
        v_cruise = u * v_path;
    } else {
        // Short segment: triangular profile (cannot reach PFL).
        const double v_peak = std::sqrt(a_path * d_max);
        t_acc = v_peak / a_path;
        t_dec = v_peak / a_path;
        t_cruise = 0.0;
        s1 = d_max / 2.0;
        s2 = d_max / 2.0;
        v_cruise = u * v_peak;
    }

    const Eigen::VectorXd q_1 = q_start_c + (s1 / d_max) * D;
    const Eigen::VectorXd q_2 = q_start_c + (s2 / d_max) * D;

    // Append one quintic sub-segment, advancing the sample counter `t` so that
    // timestamps increase by dt (wp.timestamp = dt * t inside build_p2p_traj).
    auto append_segment = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                              const Eigen::VectorXd& va, const Eigen::VectorXd& vb,
                              double T, double& t) {
        if (T <= 0.0) return;
        const Eigen::VectorXd zero = Eigen::VectorXd::Zero(7);
        build_p2p_traj(a, b, nullptr, traj, T, t, robot, dt, va, zero, vb, zero);
        t += T;
    };

    // `t` is a sample index: wp.timestamp = dt * t, so start at t_start / dt so
    // the first sample is stamped at t_start.
    double t = t_start / dt;

    // Phase 1: accelerate from steady status to cruise velocity.
    append_segment(q_start_c, q_1,
                   Eigen::VectorXd::Zero(7), v_cruise, t_acc, t);
    // Phase 2: cruise at PFL velocity.
    append_segment(q_1, q_2, v_cruise, v_cruise, t_cruise, t);
    // Phase 3: decelerate to zero velocity/acceleration at the following waypoint.
    append_segment(q_2, q_end_c, v_cruise, Eigen::VectorXd::Zero(7), t_dec, t);
}


// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point slow-down trajectory.
// ─────────────────────────────────────────────────────────────────────────────
// Instead of completely stopping the robot, this function only slows it down:
// the robot is decelerated from its current velocity to a constant peak
// velocity equal to `velocity_PFL` and then keeps following its previous
// trajectory. Returns the index of the last sample of the inserted
// slow-down segment.
// ─────────────────────────────────────────────────────────────────────────────
size_t slow_down_trajctory(Trajectory& traj, RobotModel& robot,
                           const Eigen::VectorXd& q_stop,
                           const Eigen::VectorXd& qd_stop, const Eigen::VectorXd& qdd_stop,
                           const double& t_stop, const double& stop_duration, double dt,
                           double velocity_PFL) {
    // The stop status usually references an element of traj (traj.wp[r_idx].q),
    // which is modified below: work on copies.
    const Eigen::VectorXd q_stop_c  = q_stop;
    const Eigen::VectorXd qd_stop_c = qd_stop;
    const Eigen::VectorXd qdd_stop_c = qdd_stop;

    // ── Adding slow-down waypoint ────────────────────────────────────────────────
    size_t ins = 0;
    while (ins < t_robot.size() && t_robot[ins] < t_stop) ins++;

    // Path-consistent limit point: position reached after decelerating from
    // the current velocity to velocity_PFL over stop_duration, along the
    // nominal trajectory direction toward the next waypoint.
    const double vd_norm = qd_stop_c.norm();
    const double v_target = std::min(velocity_PFL, vd_norm);
    const double s_decel = stop_duration * (vd_norm + v_target) / 2.0;
    Eigen::VectorXd q_limit_c = q_stop_c;
    if (ins < q_robot.size()) {
        const Eigen::VectorXd dq_path = q_robot[ins] - q_stop_c;
        const double dq_norm = dq_path.norm();
        if (dq_norm > 1e-12 && s_decel > 1e-12) {
            q_limit_c = q_stop_c + (dq_path / dq_norm) * std::min(s_decel, dq_norm);
        }
    }

    std::vector<Eigen::VectorXd> q_robot_temp;
    std::vector<double> t_robot_temp;
    q_robot_temp.reserve(q_robot.size() + 2);
    t_robot_temp.reserve(q_robot.size() + 2);

    // std::cout << std::endl;
    // std::cout << "t_data: ";
    // for (const auto& t : t_data) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // Offset added to the recorded timestamps of the waypoints that are
    // still to be reached. It is computed from the time actually left in
    // the interrupted segment once the restart point is inserted.
    for (size_t i = 0; i <= q_robot.size(); ++i) {
        double t_ref = t_data[t_data.size() - (q_robot.size() - i)];
        if (i == ins) {
            q_robot_temp.push_back(q_stop_c);
            q_robot_temp.push_back(q_limit_c);
            t_robot_temp.push_back(t_stop);
            t_robot_temp.push_back(t_stop + stop_duration);
        }

        if (i == q_robot.size()) break;
        i < ins ? t_robot_temp.push_back(t_robot[i]) : t_robot_temp.push_back(t_robot[i] + stop_duration);
        q_robot_temp.push_back(q_robot[i]);
    }

    q_robot = q_robot_temp;
    t_robot = t_robot_temp;

    // std::cout << std::endl;
    // std::cout << "t_robot: ";
    // for (const auto& t : t_robot) {
    //     std::cout << t << " ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;


    // ── Re-time every remaining segment to run at constant peak velocity PFL ─
    // From the slow-down point on, stretch/compress each segment so its peak
    // joint velocity equals velocity_PFL, until the end of the trajectory.
    // A zero-end-velocity quintic over time T has peak velocity 15*d/(8*T) per
    // joint (d = joint displacement), hence T = 15*max|dq|/(8*velocity_PFL).
    double t_accum = t_stop + stop_duration;  // time at q_robot[ins + 1]
    for (size_t i = ins + 1; i + 1 < q_robot_temp.size(); ++i) {
        const Eigen::VectorXd dq = q_robot_temp[i + 1] - q_robot_temp[i];
        const double dq_max = dq.cwiseAbs().maxCoeff();
        const double T = (dq_max > 1e-12 && velocity_PFL > 1e-9)
                             ? (15.0 * dq_max) / (8.0 * velocity_PFL)
                             : dt;
        t_accum += std::max(T, dt);
        t_robot_temp[i + 1] = t_accum;
    }


    // ── Partial segment assembly ────────────────────────────────────────────────
    // The waypoints before the slow-down point are unchanged, so the samples of
    // the trajectory leading up to them are kept. Only the segment that ends at
    // the inserted slow-down waypoint (ins - 1) and the following ones are
    // rebuilt, starting from the insertion index. Every segment from the
    // slow-down point onward is executed at peak velocity velocity_PFL.
    const size_t first_seg = ins > 0 ? ins - 1 : 0;
    const size_t keep = truncate_trajectory(traj, sampled_prefix(t_robot_temp, first_seg, dt));
    double t = dt * static_cast<double>(keep);

    // Velocity the robot keeps after the slow-down: same direction as the
    // current motion, with magnitude equal to velocity_PFL.
    Eigen::VectorXd v_slow = Eigen::VectorXd::Zero(qd_stop_c.size());
    if (vd_norm > 1e-9) {
        v_slow = qd_stop_c * (velocity_PFL / vd_norm);
    }

    for (size_t i = first_seg; i + 1 < q_robot_temp.size(); ++i) {
        const double t_end = t_robot_temp[i + 1] - t_robot_temp[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        if (i == ins) {
            // Decelerate from the current velocity to velocity_PFL at the
            // slow-down point instead of coming to a full stop.
            build_p2p_traj(q_robot_temp[i], q_robot_temp[i + 1], nullptr, traj, t_end, t, robot, dt,
                           qd_stop_c, qdd_stop_c, v_slow);
        }
        else if (i == ins + 1) {
            // Keep the whole remaining trajectory at velocity_PFL: start each
            // segment at velocity_PFL so the peak velocity stays constant until
            // the end of the nominal trajectory.
            build_p2p_traj(q_robot_temp[i], q_robot_temp[i + 1], nullptr, traj, t_end, t, robot, dt,
                           v_slow);
        }
        else {
            build_p2p_traj(q_robot_temp[i], q_robot_temp[i + 1], nullptr, traj, t_end, t, robot, dt);
        }
    }

    // Index of the last sample of the inserted slow-down segment: the segment
    // at index `ins` (q_stop -> q_limit_c) spans the slow-down duration.
    const size_t slow_end = sampled_prefix(t_robot_temp, ins + 1, dt);
    return slow_end > 0 ? slow_end - 1 : 0;
}
