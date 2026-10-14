#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <sstream>

#include "trajectory_utils.hpp"
#include "robot_model.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Generic CSV helpers, internal to this translation unit
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Read a CSV file into a matrix of doubles, skipping blank and non-numeric rows
// ─────────────────────────────────────────────────────────────────────────────
std::vector<std::vector<double>> read_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open " + path);

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.find_first_not_of(" \t") == std::string::npos) continue;

        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        bool ok = true;
        while (std::getline(ss, cell, ',')) {
            try { row.push_back(std::stod(cell)); }
            catch (...) { ok = false; break; }
        }
        if (ok && !row.empty()) rows.push_back(row);
    }
    return rows;
}

// ─────────────────────────────────────────────────────────────────────────────
// Load waypoints from a CSV file, requiring exactly three values per row
// ─────────────────────────────────────────────────────────────────────────────
std::vector<Eigen::Vector3d> load_waypoints(const std::string& path) {
    std::vector<Eigen::Vector3d> pts;
    for (const auto& r : read_csv(path)) {
        if (r.size() != 3)
            throw std::runtime_error(path + ": each row must have exactly 3 values (x,y,z)");
        pts.emplace_back(r[0], r[1], r[2]);
    }
    if (pts.size() < 2)
        throw std::runtime_error(path + ": need at least 2 waypoints");
    return pts;
}

// ─────────────────────────────────────────────────────────────────────────────
// Load the final time of each segment from a CSV file
// ─────────────────────────────────────────────────────────────────────────────
std::vector<double> load_time_final(const std::string& path) {
    std::vector<double> ts;
    for (const auto& r : read_csv(path)) {
        if (r.empty())
            throw std::runtime_error(path + ": no time value found");
        ts.emplace_back(r[0]);
    }
    if (ts.size() < 2)
        throw std::runtime_error(path + ": need at least 2 waypoints");
    return ts;
}

}


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
// Build a point-to-point trajectory from CSV waypoints solved with IK
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj, int N,
                               double dt, const std::string& p_csv,
                               const std::string& t_csv) {

    // ── Waypoint and time loading ───────────────────────────────────────────
    std::string trajectory_path = c_dir + "src/trajectories/test" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    if (!f) {
        throw std::runtime_error("Error: cannot open '" + trajectory_path + "'");
    }

    const std::vector<Eigen::Vector3d> robot_p = load_waypoints(trajectory_path + p_csv);
    const std::vector<double> robot_t = load_time_final(trajectory_path + t_csv);

    // ── Inverse kinematics per waypoint ─────────────────────────────────────
    Eigen::VectorXd q_base(7);
    q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(1, 1) = -1.0;
    R(2, 2) = -1.0;

    std::vector<Eigen::VectorXd> robot_q;
    robot_q.reserve(robot_p.size());
    for (size_t i = 0; i < robot_p.size(); ++i) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;
        T.translation() = robot_p[i];

        Eigen::VectorXd q;
        if (!robot.ComputeIK("panda_link8", T, q_base, &q))
            throw std::runtime_error("IK failed for waypoint " + std::to_string(i));
        robot_q.push_back(q);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Append one quintic segment to the output trajectory
    // ─────────────────────────────────────────────────────────────────────────────
    auto build_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          std::vector<double>* vnorm, Trajectory& out, const double t_end, double& t) {
        Traj tr = QuinticPolyTraj(a, b, t_end, dt, N);

        for (int k = 0; k < N; ++k) {
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
    for (size_t i = 0; i < n_seg; ++i)
        build_traj(robot_q[i], robot_q[i + 1], &v_norm, traj, robot_t[i + 1] - robot_t[i], t);

    // ── Velocity statistics ─────────────────────────────────────────────────
    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    return traj;
}
