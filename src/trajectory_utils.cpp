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
double t_gap_prev = 0.0;

// ─────────────────────────────────────────────────────────────────────────────
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, double T,
                        double dt, int n) {
    const int d = static_cast<int>(x0.size());
    Traj tr{Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n)};
    const Eigen::VectorXd dx = x1 - x0;
    for (int k = 0; k < n; ++k) {
        const double t = std::min(k * dt, T);
        const double s = t / T;
        const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
        const double h = 10 * s3 - 15 * s4 + 6 * s5;
        const double dh = (30 * s2 - 60 * s3 + 30 * s4) / T;
        const double ddh = (60 * s - 180 * s2 + 120 * s3) / (T * T);
        tr.p.col(k) = x0 + dx * h;
        tr.v.col(k) = dx * dh;
        tr.a.col(k) = dx * ddh;
    }
    return tr;
}


// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj,
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

    





    std::cout << "q_robot = " << std::endl;
    for (auto p : q_robot) std::cout << p.transpose() << std::endl;
    std::cout << "t_robot = " << std::endl;
    for (auto t : t_robot) std::cout << t << std::endl;







    // ─────────────────────────────────────────────────────────────────────────────
    // Append one quintic segment to the output trajectory
    // ─────────────────────────────────────────────────────────────────────────────
    auto build_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          std::vector<double>* vnorm, Trajectory& out, const double t_end, double& t) {
        // The segment is sampled at the control rate, so that the playback
        // takes exactly the recorded duration.
        const int n = std::max(2, static_cast<int>(std::lround(t_end / dt)));

        Traj tr = QuinticPolyTraj(a, b, t_end, dt, n);

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
    };

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
        build_traj(q_robot[i], q_robot[i + 1], &v_norm, traj, t_end, t);
    }

    if (traj.q.empty()) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory holds no motion");
    }

    // ── Velocity statistics ─────────────────────────────────────────────────
    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    std::cout << "traj q = " << std::endl;
    std::cout << traj.q[0].transpose() << std::endl;

    return traj;
}



double computeAddedTime(const Eigen::VectorXd& q_end, const Eigen::VectorXd& q_c)
{
    static const KinematicsLimits limits;

    // Upper limits (row 1) as column vectors
    const Eigen::VectorXd q_p_lim  = limits.qd_limits.row(1).transpose();
    const Eigen::VectorXd q_pp_lim = limits.qdd_limits.row(1).transpose();

    const Eigen::VectorXd dq = q_end - q_c;

    // Velocity-limited time
    const Eigen::VectorXd vect_t_vel = (15.0 * dq).cwiseQuotient(8.0 * q_p_lim);
    const double t_vel = vect_t_vel.cwiseAbs().maxCoeff();

    // Acceleration-limited time
    const Eigen::VectorXd vect_t_acc = (10.0 * dq).cwiseQuotient(std::sqrt(3.0) * q_pp_lim);
    const double t_acc = vect_t_acc.cwiseAbs().cwiseSqrt().maxCoeff();

    // Enforce minimum duration of 0.5 s
    return std::max({t_vel, t_acc, 0.5});
}




// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory restart_p2p_trajectory(RobotModel& robot, const Eigen::VectorXd& q_restart,
                                    const double& t_restart, double dt) {
    // ── Restart time ────────────────────────────────────────────────────────
    // Only the waypoints whose timestamp is strictly smaller than the restart
    // time are kept: the trajectory is rebuilt up to that point.
    if (t_restart && t_restart > 0.0) {
        std::cout << "q restart = " << q_restart.transpose() << std::endl;
        std::cout << "t restart = " << t_restart << std::endl;

        size_t ins = 0;
        while (ins < t_robot.size() && t_robot[ins] < t_restart) ins++;

        std::cout << "insert = " << ins << std::endl;

        std::vector<Eigen::VectorXd> q_robot_temp;
        std::vector<double> t_robot_temp;
        q_robot_temp.reserve(q_robot.size() + 1);
        t_robot_temp.reserve(q_robot.size() + 1);

        // Offset added to the recorded timestamps of the waypoints that are
        // still to be reached. It is computed from the time actually left in
        // the interrupted segment once the restart point is inserted.
        double t_gap = 0.0;


        




        for (size_t i = 0; i <= q_robot.size(); ++i) {
            if (i == ins) {
                q_robot_temp.push_back(q_restart);
                t_robot_temp.push_back(t_restart);

                t_gap = t_restart - t_robot[i-1]; // computeAddedTime(q_robot[i], q_restart);
            }

            if (i == q_robot.size()) break;

            q_robot_temp.push_back(q_robot[i]);
            t_robot_temp.push_back(t_robot[i] + t_gap); // - t_gap_prev);
        }

        q_robot = q_robot_temp;
        t_robot = t_robot_temp;

        t_gap_prev = t_gap;
    }    

    

    // ─────────────────────────────────────────────────────────────────────────────
    // Append one quintic segment to the output trajectory
    // ─────────────────────────────────────────────────────────────────────────────
    auto build_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          std::vector<double>* vnorm, Trajectory& out, const double t_end, double& t) {
        // The segment is sampled at the control rate, so that the playback
        // takes exactly the recorded duration.
        const int n = std::max(2, static_cast<int>(std::lround(t_end / dt)));

        Traj tr = QuinticPolyTraj(a, b, t_end, dt, n);

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
    };

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
        build_traj(q_robot[i], q_robot[i + 1], &v_norm, traj, t_end, t);
    }

    // ── Velocity statistics ─────────────────────────────────────────────────
    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    std::cout << "traj q = " << std::endl;
    std::cout << traj.q[0].transpose() << std::endl;

    return traj;
}
