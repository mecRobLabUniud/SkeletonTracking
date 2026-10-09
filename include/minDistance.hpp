#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Result of the point-to-segment distance computation
// ─────────────────────────────────────────────────────────────────────────────
struct MinRosResult {
    double minros;
    Eigen::Vector3d roa;
    Eigen::Vector3d rob;
    Eigen::Vector3d rba;
    double s;
};

// ─────────────────────────────────────────────────────────────────────────────
// Minimum distance between point ro and the segment [ra, rb]
// ─────────────────────────────────────────────────────────────────────────────
inline MinRosResult minsros(const Eigen::Vector3d& ra,
                             const Eigen::Vector3d& rb,
                             const Eigen::Vector3d& ro) {
    MinRosResult out;
    out.rba = rb - ra;
    out.roa = ro - ra;
    out.rob = ro - rb;
    out.s = out.roa.dot(out.rba) / out.rba.dot(out.rba);

    Eigen::Vector3d rs = ra + out.s * out.rba;
    Eigen::Vector3d ros = ro - rs;

    if (out.s >= 0.0 && out.s <= 1.0) {
        out.minros = ros.norm();
    } else {
        out.minros = std::min(out.roa.norm(), out.rob.norm());
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Clearance from a safety cylinder of radius delta/2 around segment [ra, rb]
// to the obstacle point ro
// ─────────────────────────────────────────────────────────────────────────────
inline double minsSSM(const Eigen::Vector3d& ra,
                       const Eigen::Vector3d& rb,
                       const Eigen::Vector3d& ro,
                       double delta) {
    MinRosResult m = minsros(ra, rb, ro);

    if (m.minros >= delta / 2.0) {
        return m.minros - delta / 2.0;
    }

    double omega_0_prime = m.roa.dot(m.roa);
    double omega_1 = -2.0 * m.roa.dot(m.rba);
    double omega_2 = m.rba.dot(m.rba);
    double omega_0 = omega_0_prime - (delta * delta) / 4.0;

    double disc = omega_1 * omega_1 - 4.0 * omega_0 * omega_2;
    double sq_delta = std::sqrt(std::max(disc, 0.0));

    double s1 = (-omega_1 + sq_delta) / (2.0 * omega_2);
    double s2 = (-omega_1 - sq_delta) / (2.0 * omega_2);

    bool s1_in = (s1 <= 1.0 && s1 >= 0.0);
    bool s2_in = (s2 <= 1.0 && s2 >= 0.0);

    if (s1_in || s2_in) {
        return 0.0;
    }

    return std::min(std::abs(m.roa.norm() - delta / 2.0),
                     std::abs(m.rob.norm() - delta / 2.0));
}
