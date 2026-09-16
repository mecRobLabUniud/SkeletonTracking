#include <fstream>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "trajectory_utils.hpp"

// ── CSV I/O ────────────────────────────────────────────────────────────────
std::vector<std::array<double, 7>> load_trajectory_CSV(const std::string& path) {
    std::ifstream f(path);

    std::vector<std::array<double, 7>> traj;
    std::string line;

    while (std::getline(f, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::array<double, 7> wp;
        std::stringstream ss(line);
        std::string token;
        int j = 0;

        while (std::getline(ss, token, ',') && j < 7) {
            try {
                wp[j++] = std::stod(token);
            } catch (const std::exception&) {
                throw std::runtime_error("Bad value at line: " + line);
            }
        }

        if (j != 7)
            throw std::runtime_error("Expected 7 values, got "
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

    std::cout << "========== v Low ==========\n" << v_low.transpose() << "\n";
    std::cout << "========== a Low ==========\n" << a_low.transpose() << "\n";

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

    Trajectory out;
    out.q.resize(n_high);
    out.qd.resize(n_high);
    out.qdd.resize(n_high);

    for (int j = 0; j < 7; ++j) {
        Eigen::VectorXd q_low(n);
        for (int i = 0; i < n; ++i)
            q_low(i) = traj_low[i][j];

        Eigen::VectorXd q_high, v_high, a_high;
        quintic_spline_interp_full(t_low, q_low, t_high, q_high, v_high, a_high);

        for (int i = 0; i < n_high; ++i) {
            out.q[i][j] = q_high(i);
            out.qd[i][j] = v_high(i);
            out.qdd[i][j] = a_high(i);
        }
    }

    return out;
}