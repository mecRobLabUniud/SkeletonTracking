#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>

#include "robot_model.hpp"

const std::array<std::array<int, 2>, 14> MP_SKELETON = {{
    {0, 0}
}};

struct DistanceResult {
    double length;
    Eigen::Vector3d c_h;
    Eigen::Vector3d c_r;
    int ind_h;
};

DistanceResult segm_to_segm_distance(
    const Eigen::Vector3d& P1,
    const Eigen::Vector3d& Q1,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2);

std::optional<DistanceResult> human_to_segm_distance(
    const std::vector<Eigen::Vector3d>& skeleton,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2);

std::optional<DistanceResult> human_to_robot_distance(
    const std::vector<Eigen::Vector3d>& skeleton,
    const RobotModel& robot, 
    const Eigen::VectorXd& q);