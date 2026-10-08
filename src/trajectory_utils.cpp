#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "trajectory_utils.hpp"
#include "robot_model.hpp"

// ── CSV I/O ────────────────────────────────────────────────────────────────
std::vector<std::array<double, 3>> load_trajectory_CSV(const std::string& path) {
    std::ifstream f(path);

    std::vector<std::array<double, 3>> traj;
    std::string line;

    while (std::getline(f, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::array<double, 3> wp;
        std::stringstream ss(line);
        std::string token;
        int j = 0;

        while (std::getline(ss, token, ',') && j < 3) {
            try {
                wp[j++] = std::stod(token);
            } catch (const std::exception&) {
                throw std::runtime_error("Bad value at line: " + line);
            }
        }

        if (j != 3)
            throw std::runtime_error("Expected 3 values, got "
                                     + std::to_string(j)
                                     + " at line: " + line);
        traj.push_back(wp);
    }

    std::cout << "Loaded " << traj.size() << " waypoints from " << path << "\n";
    return traj;
}

void save_trajectory_CSV(const std::string& path,
                       const std::vector<std::array<double, 7>>& traj) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot write: " + path);

    f << std::fixed << std::setprecision(9);
    f << "# q1,q2,q3,q4,q5,q6,q7\n";
    for (const auto& wp : traj) {
        for (int j = 0; j < 7; ++j) {
            f << wp[j];
            if (j < 6) f << ",";
        }
        f << "\n";
    }

    std::cout << "Saved " << traj.size() << " waypoints to " << path << "\n";
}

std::vector<double> load_timestamps_CSV(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);

    std::vector<double> t;
    std::string line;

    while (std::getline(f, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        double wp;
        std::stringstream ss(line);
        std::string token;
        int j = 0;

        while (std::getline(ss, token, ',') && j++ < 1) {
            try {
                wp = std::stod(token);
            } catch (const std::exception&) {
                throw std::runtime_error("Bad value at line: " + line);
            }
        }

        if (j != 1)
            throw std::runtime_error("Expected 1 value, got "
                                     + std::to_string(j)
                                     + " at line: " + line);
        t.push_back(wp);
    }

    std::cout << "Loaded " << t.size() << " waypoints from " << path << "\n";
    return t;
}


// ── Generic CSV helpers (internal) ─────────────────────────────────────────
namespace {

std::vector<std::vector<double>> read_csv(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open " + path);

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();   // Windows line endings
        if (line.find_first_not_of(" \t") == std::string::npos) continue;  // blank line

        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        bool ok = true;
        while (std::getline(ss, cell, ',')) {
            try { row.push_back(std::stod(cell)); }
            catch (...) { ok = false; break; }   // non-numeric -> treat as header and skip
        }
        if (ok && !row.empty()) rows.push_back(row);
    }
    return rows;
}

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

}  // namespace


// ── Quintic polynomial trajectory between two points ───────────────────────
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


Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir, int n_traj, int N,
                               double dt, const std::string& p_csv,
                               const std::string& t_csv) {

    std::string trajectory_path = c_dir + "src/trajectories/test" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    if (!f) {
        throw std::runtime_error("Error: cannot open '" + trajectory_path + "'");
    }

    const std::vector<Eigen::Vector3d> robot_p = load_waypoints(trajectory_path + p_csv);
    const std::vector<double> robot_t = load_time_final(trajectory_path + t_csv);

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

    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    return traj;
}


// ── Finite-difference velocity estimation ──────────────────────────────────
// Central differences for interior points, one-sided for endpoints
Eigen::VectorXd estimate_velocities(const Eigen::VectorXd& t,
                                    const Eigen::VectorXd& q) {
    int n = q.size();
    Eigen::VectorXd v(n);

    // Endpoints: one-sided (clamped to zero for start/stop motions)
    v(0)     = 0.0;
    v(n - 1) = 0.0;

    // Interior: central differences
    for (int i = 1; i < n - 1; ++i)
        v(i) = (q(i + 1) - q(i - 1)) / (t(i + 1) - t(i - 1));

    return v;
}

// ── Finite-difference acceleration estimation ───────────────────────────────
Eigen::VectorXd estimate_accelerations(const Eigen::VectorXd& t,
                                       const Eigen::VectorXd& q) {
    int n = q.size();
    Eigen::VectorXd a(n);

    // Endpoints: clamped to zero (robot starts and ends at rest)
    a(0)     = 0.0;
    a(n - 1) = 0.0;

    // Interior: central second differences
    for (int i = 1; i < n - 1; ++i) {
        double h0 = t(i)     - t(i - 1);
        double h1 = t(i + 1) - t(i);
        a(i) = 2.0 * ((q(i + 1) - q(i)) / h1 - (q(i) - q(i - 1)) / h0)
                   / (h0 + h1);
    }

    return a;
}


// ── Per-joint quintic spline interpolation (pos + vel + acc) ───────────────
void quintic_spline_interp_full(const Eigen::VectorXd& t_low,
                                 const Eigen::VectorXd& q_low,
                                 const Eigen::VectorXd& t_high,
                                 Eigen::VectorXd& q_high,
                                 Eigen::VectorXd& v_high,
                                 Eigen::VectorXd& a_high) {
    int n = t_low.size();
    int m = t_high.size();

    Eigen::VectorXd v_low = estimate_velocities(t_low, q_low);
    Eigen::VectorXd a_low = estimate_accelerations(t_low, q_low);

    q_high.resize(m);
    v_high.resize(m);
    a_high.resize(m);

    int seg = 0;
    for (int i = 0; i < m; ++i) {
        double t = t_high(i);
        while (seg < n - 2 && t > t_low(seg + 1)) ++seg;

        double h = t_low(seg + 1) - t_low(seg);
        double s = (t - t_low(seg)) / h;

        double q0 = q_low(seg),   v0 = v_low(seg),   a0 = a_low(seg);
        double q1 = q_low(seg+1), v1 = v_low(seg+1), a1 = a_low(seg+1);

        q_high(i) = quintic_hermite(s, h, q0, v0, a0, q1, v1, a1);
        v_high(i) = quintic_hermite_vel(s, h, q0, v0, a0, q1, v1, a1);
        a_high(i) = quintic_hermite_acc(s, h, q0, v0, a0, q1, v1, a1);
    }
}


// ── Main interpolation entry point ──────────────────────────────────────────
Trajectory interpolate_to_1kHz_full(
        const std::vector<std::array<double, 7>>& traj_low,
        std::vector<double> time_low) {

    int    n        = traj_low.size();
    double duration = time_low[n-1];
    int    n_high   = static_cast<int>(duration * 1000.0);

    Eigen::VectorXd t_low(n), t_high(n_high);
    for (int i = 0; i < n;      ++i) t_low(i)  = time_low[i];
    for (int i = 0; i < n_high; ++i) t_high(i) = i / 1000.0;

    std::vector<std::array<double, 7>> q;
    q.resize(n_high);
    std::vector<std::array<double, 7>> qd;
    qd.resize(n_high);
    std::vector<std::array<double, 7>> qdd;
    qdd.resize(n_high);

    for (int j = 0; j < 7; ++j) {
        Eigen::VectorXd q_low(n);
        for (int i = 0; i < n; ++i)
            q_low(i) = traj_low[i][j];

        Eigen::VectorXd q_high, v_high, a_high;
        quintic_spline_interp_full(t_low, q_low, t_high, q_high, v_high, a_high);

        for (int i = 0; i < n_high; ++i) {
            q[i][j] = q_high(i);
            qd[i][j] = v_high(i);
            qdd[i][j] = a_high(i);
        }
    }

    Trajectory out;
    out.q.resize(q.size());
    out.qd.resize(qd.size());
    out.qdd.resize(qdd.size());
    std::transform(q.begin(), q.end(), out.q.begin(),
        [](std::array<double, 7>& x) {
            return Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
        });
    std::transform(qd.begin(), qd.end(), out.qd.begin(),
        [](std::array<double, 7>& x) {
            return Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
        });
    std::transform(qdd.begin(), qdd.end(), out.qdd.begin(),
        [](std::array<double, 7>& x) {
            return Eigen::Map<Eigen::VectorXd>(x.data(), x.size());
        });

    // out.q = Eigen::Map<Eigen::VectorXd>(q.data(), q.size());
    // out.qd = Eigen::Map<Eigen::VectorXd>(qd.data(), qd.size());
    // out.qdd = Eigen::Map<Eigen::VectorXd>(qdd.data(), qdd.size());

    return out;
}