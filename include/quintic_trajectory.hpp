#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

// struct Trajectory {
//     std::vector<Eigen::VectorXd> q;   // position
//     std::vector<Eigen::VectorXd> qd;  // velocity
//     std::vector<Eigen::VectorXd> qdd; // acceleration
// };

/**
 * Interpolates a low-rate joint trajectory (waypoints + timestamps) with
 * quintic polynomials, sampled at 1 kHz (dt = 0.001 s by default).
 *
 * - Boundary conditions: zero velocity and acceleration at the first and
 *   last waypoint.
 * - Interior waypoints (if any): acceleration = 0, velocity = harmonic mean
 *   of the adjacent segment slopes (0 if the slope changes sign). This gives
 *   a C2-continuous trajectory without overshoot between waypoints.
 * - With only 2 waypoints this is the classic rest-to-rest quintic.
 * - stop_at_waypoints = true forces zero velocity at every waypoint.
 *
 * Timestamps are absolute; the output starts at t_low.front() and always
 * ends exactly at t_low.back().
 */
inline Trajectory interpolateQuintic(
    const std::vector<std::array<double, 7>>& traj_low,
    const std::vector<double>& t_low,
    double dt = 0.001,
    bool stop_at_waypoints = false)
{
    constexpr int DOF = 7;
    const size_t n = traj_low.size();

    if (n < 2 || t_low.size() != n)
        throw std::invalid_argument("Need >= 2 waypoints and one timestamp per waypoint");
    if (dt <= 0.0)
        throw std::invalid_argument("dt must be positive");
    for (size_t i = 1; i < n; ++i)
        if (t_low[i] <= t_low[i - 1])
            throw std::invalid_argument("Timestamps must be strictly increasing");

    // --- Knot velocities (n x DOF) ---
    std::vector<std::array<double, DOF>> v(n);
    for (auto& row : v) row.fill(0.0);

    if (!stop_at_waypoints) {
        for (size_t i = 1; i + 1 < n; ++i) {
            const double h0 = t_low[i] - t_low[i - 1];
            const double h1 = t_low[i + 1] - t_low[i];
            for (int j = 0; j < DOF; ++j) {
                const double s0 = (traj_low[i][j] - traj_low[i - 1][j]) / h0;
                const double s1 = (traj_low[i + 1][j] - traj_low[i][j]) / h1;
                v[i][j] = (s0 * s1 > 0.0) ? 2.0 * s0 * s1 / (s0 + s1) : 0.0;
            }
        }
    }

    // --- Sampling ---
    const double t_start = t_low.front();
    const double t_end = t_low.back();
    const size_t N = static_cast<size_t>(std::floor((t_end - t_start) / dt + 1e-9)) + 1;

    Trajectory traj;
    traj.q.reserve(N + 1);
    traj.qd.reserve(N + 1);
    traj.qdd.reserve(N + 1);

    size_t seg = 0;
    auto sample = [&](double t) {
        t = std::min(t, t_end);
        while (seg + 2 < n && t >= t_low[seg + 1]) ++seg;

        const double h = t_low[seg + 1] - t_low[seg];
        const double tau = std::clamp(t - t_low[seg], 0.0, h);

        Eigen::VectorXd q(DOF), qd(DOF), qdd(DOF);
        for (int j = 0; j < DOF; ++j) {
            const double q0 = traj_low[seg][j],     q1 = traj_low[seg + 1][j];
            const double v0 = v[seg][j],            v1 = v[seg + 1][j];
            const double a0 = 0.0,                  a1 = 0.0; // knot accelerations

            const double h2 = h * h, h3 = h2 * h, h4 = h3 * h, h5 = h4 * h;

            const double c0 = q0;
            const double c1 = v0;
            const double c2 = 0.5 * a0;
            const double c3 = (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * h
                               - (3.0 * a0 - a1) * h2) / (2.0 * h3);
            const double c4 = (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * h
                               + (3.0 * a0 - 2.0 * a1) * h2) / (2.0 * h4);
            const double c5 = (12.0 * (q1 - q0) - 6.0 * (v1 + v0) * h
                               - (a0 - a1) * h2) / (2.0 * h5);

            q(j)   = c0 + tau * (c1 + tau * (c2 + tau * (c3 + tau * (c4 + tau * c5))));
            qd(j)  = c1 + tau * (2.0 * c2 + tau * (3.0 * c3 + tau * (4.0 * c4 + tau * 5.0 * c5)));
            qdd(j) = 2.0 * c2 + tau * (6.0 * c3 + tau * (12.0 * c4 + tau * 20.0 * c5));
        }
        traj.q.push_back(std::move(q));
        traj.qd.push_back(std::move(qd));
        traj.qdd.push_back(std::move(qdd));
    };

    for (size_t k = 0; k < N; ++k)
        sample(t_start + static_cast<double>(k) * dt);

    // Make sure the trajectory ends exactly on the last waypoint
    if (t_end - (t_start + static_cast<double>(N - 1) * dt) > 1e-9)
        sample(t_end);

    return traj;
}