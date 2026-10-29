/*
░█▄█░▀█▀░█▀█░░░░░█▀▄░▀█▀░█▀▀░▀█▀░█▀█░█▀█░█▀▀░█▀▀░░░░░█▀▀░█▀█░█░░░█▀▀░█░█░█░░░█▀█░▀█▀░▀█▀░█▀█░█▀█
░█░█░░█░░█░█░░░░░█░█░░█░░▀▀█░░█░░█▀█░█░█░█░░░█▀▀░░░░░█░░░█▀█░█░░░█░░░█░█░█░░░█▀█░░█░░░█░░█░█░█░█
░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀░▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀

min_distance_calculation.cpp - Minimum-distance routines between the human
skeleton segments and the robot link segments.
*/

#include "min_distance_calculation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

DistanceResult segm_to_segm_distance(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& q1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2)
{
    Eigen::Vector3d d1 = q1 - p1;
    Eigen::Vector3d d2 = q2 - p2;
    Eigen::Vector3d r  = p1 - p2;

    double a = d1.dot(d1);
    double b = d1.dot(d2);
    double c = d1.dot(r);
    double e = d2.dot(d2);
    double f = d2.dot(r);

    double d = a * e - b * b;
    if (std::abs(d) < 1e-12) {
        // Segments are nearly parallel; fallback to endpoints
        double dist_min = std::numeric_limits<double>::infinity();
        Eigen::Vector3d c_h_min = p1;
        Eigen::Vector3d c_r_min = p2;

        const std::array<Eigen::Vector3d, 2> pts1 = {p1, q1};
        const std::array<Eigen::Vector3d, 2> pts2 = {p2, q2};

        for (const auto& pa : pts1) {
            for (const auto& pb : pts2) {
                double dist = (pa - pb).norm();
                if (dist < dist_min) {
                    dist_min = dist;
                    c_h_min = pa;
                    c_r_min = pb;
                }
            }
        }

        return {dist_min, c_h_min, c_r_min, -1};
    }

    double s = std::clamp<double>(((b * f - c * e) / d), 0, 1);
    double t = (b * s + f) / e;

    if (t < 0.0) {
        t = 0.0;
        s = std::clamp<double>((-c / a), 0, 1);
    } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp<double>(((b - c) / a), 0, 1);
    }

    Eigen::Vector3d c_h = p1 + d1 * s;
    Eigen::Vector3d c_r = p2 + d2 * t;

    return {(c_h - c_r).norm(), c_h, c_r, -1};
}


std::optional<DistanceResult> human_to_segm_distance(
    const Skeleton& skeleton,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2) {
    std::optional<DistanceResult> best;

    for (size_t i = 0; i < MP_SKELETON.size(); ++i) {
        const int idx1 = MP_SKELETON[i][0];
        const int idx2 = MP_SKELETON[i][1];

        // Guard against a skeleton shorter than the merged 21-point layout
        if (idx1 < 0 || idx2 < 0 ||
            idx1 >= static_cast<int>(skeleton.size()) ||
            idx2 >= static_cast<int>(skeleton.size())) {
            continue;
        }

        const Eigen::Vector3d& p1 = skeleton[idx1];
        const Eigen::Vector3d& q1 = skeleton[idx2];

        // Skip segments with NaNs
        if (p1.hasNaN() || q1.hasNaN()) continue;

        DistanceResult dist = segm_to_segm_distance(p1, q1, p2, q2);
        dist.ind_h = static_cast<int>(i);

        if (!best || dist.length < best->length) {
            best = dist;
        }
    }

    return best;
}


std::optional<DistanceResult> human_to_robot_distance(
    const Skeleton& skeleton,
    const RobotModel& robot,
    const Eigen::VectorXd& q) {
    const Eigen::Vector3d r0 = robot.GetJointPose("panda_link0", q).translation();
    const Eigen::Vector3d r1 = robot.GetJointPose("panda_link1", q).translation();
    const Eigen::Vector3d r2 = robot.GetJointPose("panda_link2", q).translation();
    const Eigen::Vector3d r3 = robot.GetJointPose("panda_link3", q).translation();
    const Eigen::Vector3d r4 = robot.GetJointPose("panda_link4", q).translation();
    const Eigen::Vector3d r5 = robot.GetJointPose("panda_link5", q).translation();
    const Eigen::Vector3d r7 = robot.GetJointPose("panda_link7", q).translation();
    const Eigen::Vector3d r8 = robot.GetJointPose("panda_link8", q).translation();

    const std::array<std::optional<DistanceResult>, 4> candidates = {
        human_to_segm_distance(skeleton, r0, r1),
        human_to_segm_distance(skeleton, r2, r3),
        human_to_segm_distance(skeleton, r4, r5),
        human_to_segm_distance(skeleton, r7, r8)
    };

    std::optional<DistanceResult> best;
    for (const auto& candidate : candidates) {
        if (candidate && (!best || candidate->length < best->length)) {
            best = candidate;
        }
    }

    return best;
}


double minsSSM(const Eigen::Vector3d& ra,
               const Eigen::Vector3d& rb,
               const Eigen::Vector3d& ro,
               double delta) {
    const Eigen::Vector3d rba = rb - ra;
    const Eigen::Vector3d roa = ro - ra;
    const Eigen::Vector3d rob = ro - rb;

    const double s = roa.dot(rba) / rba.dot(rba);
    const double minros = (s >= 0.0 && s <= 1.0)
        ? (ro - (ra + s * rba)).norm()
        : std::min(roa.norm(), rob.norm());

    if (minros >= delta / 2.0) {
        return minros - delta / 2.0;
    }

    const double omega_0_prime = roa.dot(roa);
    const double omega_1 = -2.0 * roa.dot(rba);
    const double omega_2 = rba.dot(rba);
    const double omega_0 = omega_0_prime - (delta * delta) / 4.0;

    const double disc = omega_1 * omega_1 - 4.0 * omega_0 * omega_2;
    const double sq_delta = std::sqrt(std::max(disc, 0.0));

    const double s1 = (-omega_1 + sq_delta) / (2.0 * omega_2);
    const double s2 = (-omega_1 - sq_delta) / (2.0 * omega_2);

    const bool s1_in = (s1 <= 1.0 && s1 >= 0.0);
    const bool s2_in = (s2 <= 1.0 && s2 >= 0.0);

    if (s1_in || s2_in) {
        return 0.0;
    }

    return std::min(std::abs(roa.norm() - delta / 2.0),
                    std::abs(rob.norm() - delta / 2.0));
}
