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
// takes exactly the recorded time.
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj, int N,
                               double dt, const std::string& p_csv,
                               const std::string& t_csv) {

    // ── Waypoint and time loading ───────────────────────────────────────────
    const fs::path traj_dir =
        fs::path(c_dir) / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    TrajectoryData data;
    if (p_csv.empty() && t_csv.empty()) {
        // The waypoint format (Cartesian or joint space) is detected from the
        // files that are present in the trajectory directory.
        data = loadTrajectorySet(traj_dir);
    } else {
        data = loadTrajectory(traj_dir,
                              p_csv.empty() ? std::string("p_ref.csv") : p_csv,
                              t_csv.empty() ? std::string("t_ref.csv") : t_csv);
    }

    const std::vector<double>& robot_t = data.t;

    if (data.p.empty() && data.q.empty()) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory holds no waypoints");
    }
    if (robot_t.size() < 2) {
        throw std::runtime_error(traj_dir.string() + ": the trajectory needs at least 2 waypoints");
    }

    // ── Joint waypoints ─────────────────────────────────────────────────────
    // Cartesian waypoints are solved with the inverse kinematics, joint
    // waypoints are already configurations and are used as they are.
    std::vector<Eigen::VectorXd> robot_q;
    if (data.isJointSpace()) {
        robot_q = data.q;
        std::cout << "Loaded " << robot_q.size() << " joint-space waypoints from "
                  << traj_dir << std::endl;
    }
    else {
        Eigen::VectorXd q_base(7);
        q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R(1, 1) = -1.0;
        R(2, 2) = -1.0;

        robot_q.reserve(data.p.size());
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
                    ok = robot.ComputeIK("panda_link8", T, robot_q.back(), &q);
                }
                if (!ok) {
                    throw std::runtime_error("IK failed for waypoint " + std::to_string(i) +
                                             " of " + traj_dir.string());
                }
            }
            robot_q.push_back(q);
        }
        std::cout << "Loaded " << robot_q.size() << " Cartesian waypoints from "
                  << traj_dir << " and solved the IK for each of them" << std::endl;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Append one quintic segment to the output trajectory
    // ─────────────────────────────────────────────────────────────────────────────
    auto build_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          std::vector<double>* vnorm, Trajectory& out, const double t_end, double& t) {
        // The segment is sampled at the control rate, so that the playback
        // takes exactly the recorded duration.
        const int n = std::max(2, static_cast<int>(std::lround(t_end / dt)) + 1);

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
    const size_t n_seg = robot_q.size() - 1;
    std::vector<double> v_norm;
    Trajectory traj;
    traj.q.reserve(N * n_seg);
    traj.qd.reserve(N * n_seg);
    traj.qdd.reserve(N * n_seg);
    traj.p.reserve(N * n_seg);
    traj.pd.reserve(N * n_seg);
    traj.t.reserve(N * n_seg);

    double t = 0;
    for (size_t i = 0; i < n_seg; ++i) {
        const double t_end = robot_t[i + 1] - robot_t[i];
        if (t_end <= 0.0) {
            // A segment with no duration carries no motion: skip it instead of
            // building a degenerate polynomial (which would divide by zero).
            std::cout << "Skipping segment " << i << ": its duration is not positive"
                      << std::endl;
            continue;
        }
        build_traj(robot_q[i], robot_q[i + 1], &v_norm, traj, t_end, t);
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
