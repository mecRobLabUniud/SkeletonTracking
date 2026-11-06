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

        out.q.push_back(q);
        out.qd.push_back(qd);
        out.qdd.push_back(qdd);
        out.p.push_back(robot.GetJointPose("panda_link8", q).translation());
        out.pd.push_back(v.head<3>());
        out.t.push_back(dt * t++);
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
    traj.q.reserve(N);
    traj.qd.reserve(N);
    traj.qdd.reserve(N);
    traj.p.reserve(N);
    traj.pd.reserve(N);
    traj.t.reserve(N);

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

    if (traj.q.empty()) {
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
    n_keep = std::min(n_keep, traj.t.size());
    traj.q.resize(n_keep);
    traj.qd.resize(n_keep);
    traj.qdd.resize(n_keep);
    traj.p.resize(n_keep);
    traj.pd.resize(n_keep);
    traj.t.resize(n_keep);
    return n_keep;
}

}  // namespace


// ─────────────────────────────────────────────────────────────────────────────
// Rebuild a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
void restart_trajectory(Trajectory& traj, RobotModel& robot,
                        const Eigen::VectorXd& q_restart,
                        const double& t_restart, double dt) {
    // The restart configuration usually references an element of traj
    // (traj.q[r_idx]), which is modified below: work on a copy.
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

    std::cout << std::endl;
    std::cout << "t_data: ";
    for (const auto& t : t_data) {
        std::cout << t << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

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

    std::cout << std::endl;
    std::cout << "t_robot: ";
    for (const auto& t : t_robot) {
        std::cout << t << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

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
// Build a point-to-point emergency stop trajectory
// ─────────────────────────────────────────────────────────────────────────────
size_t stop_trajectory(Trajectory& traj, RobotModel& robot,
                       const Eigen::VectorXd& q_stop,
                       const Eigen::VectorXd& qd_stop, const Eigen::VectorXd& qdd_stop,
                       const double& t_stop, const double& stop_duration, double dt) {
    // The stop state usually references an element of traj (traj.q[r_idx]),
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

    std::cout << std::endl;
    std::cout << "t_data: ";
    for (const auto& t : t_data) {
        std::cout << t << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

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

    std::cout << std::endl;
    std::cout << "t_robot: ";
    for (const auto& t : t_robot) {
        std::cout << t << " ";
    }
    std::cout << std::endl;
    std::cout << std::endl;


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
