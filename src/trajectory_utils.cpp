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

namespace fs = std::filesystem;


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
//
// The waypoints live in <c_dir>/src/trajectories/test<n_traj>/ and are given
// either in Cartesian coordinates (p_ref.csv, solved here with the inverse
// kinematics) or in joint coordinates (q_ref.csv, used as they are). The
// cumulative time of each waypoint (t_ref.csv) gives the duration of every
// segment, so each segment is sampled at the control rate and the playback
// takes exactly the recorded time. When a restart time is given, the waypoints
// reached before it are kept; when a restart configuration is given as well,
// it replaces the first kept waypoint and the duration of the first segment is
// scaled with the ratio of the new to the recorded path length, so that its
// playback speed is preserved.
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj,
                               double dt, const std::optional<Eigen::VectorXd>& q_restart,
                               const std::optional<double>& t_restart) {

    // ── Waypoint and time loading ───────────────────────────────────────────
    const fs::path traj_dir =
        fs::path(c_dir) / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    // The waypoint format (Cartesian or joint space) is detected from the
    // files that are present in the trajectory directory.
    TrajectoryData data = loadTrajectorySet(traj_dir);
    TrajectoryData temp = data;

    // ── Restart time ────────────────────────────────────────────────────────
    // Only the waypoints whose timestamp is strictly smaller than the restart
    // time are kept: the trajectory is rebuilt up to that point.
    if (t_restart) {

        std::cout << "t restart = " << *t_restart << std::endl;

        size_t del = 0;
        while (del < data.t.size() && data.t[del] < *t_restart) del++;

        std::cout << "delete = " << del << std::endl;

        if (!data.q.empty()) {
            temp.q.resize(temp.t.size() - del);
            temp.q = std::vector<Eigen::VectorXd>(data.q.begin() + del, data.q.begin() + data.q.size());
        }
        if (!data.p.empty()) {
            temp.p.resize(temp.t.size() - del);
            temp.p = std::vector<Eigen::Vector3d>(data.p.begin() + del, data.p.begin() + data.p.size());
        }
        temp.t.resize(temp.t.size() - del);
        temp.t = std::vector<double>(data.t.begin() + del, data.t.begin() + data.t.size());
    }

    std::cout << "p temp = " << std::endl;
    for (auto p : temp.p) std::cout << p.transpose() << std::endl;
    std::cout << "t temp = " << std::endl;
    for (auto t : temp.t) std::cout << t << std::endl;

    if (temp.p.empty() && temp.q.empty()) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory holds no waypoints");
    }
    if (temp.t.size() < 2) {
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







    std::vector<Eigen::VectorXd> q_robot;
    std::vector<double>& t_robot = temp.t;

    if (q_restart && q_restart->size() > 0) {
        if (q_restart->size() != q_robot.front().size()) {
            throw std::runtime_error(
                traj_dir.string() + ": the restart configuration must have " +
                std::to_string(q_robot.front().size()) + " values");
        }

        q_robot.push_back(q_restart);
        t_robot.push_back(t_restart);
    }








    if (temp.isJointSpace()) {
        q_robot = temp.q;
        std::cout << "Loaded " << q_robot.size() << " joint-space waypoints from "
                  << traj_dir << std::endl;
    }
    else {
        q_robot.reserve(temp.p.size());
        for (size_t i = 0; i < temp.p.size(); ++i) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = R;
            T.translation() = temp.p[i];

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
        double t_end = t_robot[i + 1] - t_robot[i];
        if (i == 0) t_end *= first_segment_scale;
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

    return traj;
}
