#include "min_distance_calculation.hpp"

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

        DistanceResult res;
        res.length = dist_min;
        res.c_h = c_h_min;
        res.c_r = c_r_min;
        res.ind_h = -1;
        return res;
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
    double distance = (c_h - c_r).norm();

    DistanceResult res;
    res.length = distance;
    res.c_h = c_h;
    res.c_r = c_r;
    res.ind_h = -1;
    return res;
}


std::optional<DistanceResult> human_to_segm_distance(
    const std::vector<Eigen::Vector3d>& skeleton,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& q2) {
    bool first = true;
    double min_dist = 0.0;
    Eigen::Vector3d c_h_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d c_r_min = Eigen::Vector3d::Zero();
    int ind_h = -1;

    for (size_t i = 0; i < MP_SKELETON.size(); ++i) {
        int idx1 = MP_SKELETON[i][0];
        int idx2 = MP_SKELETON[i][1];

        const Eigen::Vector3d& p1 = skeleton[idx1];
        const Eigen::Vector3d& q1 = skeleton[idx2];

        // Skip segments with NaNs
        if (p1.hasNaN() || q1.hasNaN()) continue;

        DistanceResult dist = segm_to_segm_distance(p1, q1, p2, q2);

        if (first) {
            min_dist = dist.length;
            c_h_min = dist.c_h;
            c_r_min = dist.c_r;
            ind_h = static_cast<int>(i);
            first = false;
        } else {
            if (dist.length < min_dist) {
                min_dist = dist.length;
                c_h_min = dist.c_h;
                c_r_min = dist.c_r;
                ind_h = static_cast<int>(i);
            }
        }
    }

    if (first) return std::nullopt;

    DistanceResult res;
    res.length = min_dist;
    res.c_h = c_h_min;
    res.c_r = c_r_min;
    res.ind_h = ind_h;
    return res;
}


std::optional<DistanceResult> human_to_robot_distance(
    const std::vector<Eigen::Vector3d>& skeleton,
    const RobotModel& robot, 
    const Eigen::VectorXd& q) {
    Eigen::Vector3d r0 = robot.GetJointPose("panda_link0", q).translation();
    Eigen::Vector3d r1 = robot.GetJointPose("panda_link1", q).translation();
    Eigen::Vector3d r2 = robot.GetJointPose("panda_link2", q).translation();
    Eigen::Vector3d r3 = robot.GetJointPose("panda_link3", q).translation();
    Eigen::Vector3d r4 = robot.GetJointPose("panda_link4", q).translation();
    Eigen::Vector3d r5 = robot.GetJointPose("panda_link5", q).translation();
    Eigen::Vector3d r7 = robot.GetJointPose("panda_link7", q).translation();
    Eigen::Vector3d r8 = robot.GetJointPose("panda_link8", q).translation();

    std::vector<double> a;
    std::optional<DistanceResult> a0 = human_to_segm_distance(skeleton, r0, r1);
    std::optional<DistanceResult> a1 = human_to_segm_distance(skeleton, r2, r3);
    std::optional<DistanceResult> a2 = human_to_segm_distance(skeleton, r4, r5);
    std::optional<DistanceResult> a3 = human_to_segm_distance(skeleton, r7, r8);

    if (a0) a.push_back(a0->length);
    if (a1) a.push_back(a1->length);
    if (a2) a.push_back(a2->length);
    if (a3) a.push_back(a3->length);

    if (a.empty()) return std::nullopt;

    int min_dist_index = std::min_element(a.begin(), a.end()) - a.begin();

    switch (min_dist_index) {
        case 0:
            return a0;
        case 1:
            return a1;
        case 2:
            return a2;
        case 3:
            return a3;
        default:
            return std::nullopt;
    }
}