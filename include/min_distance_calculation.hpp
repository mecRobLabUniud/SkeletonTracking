#pragma once
#include <Eigen/Core>
#include <array>
#include <optional>
#include <vector>

#include "robot_model.hpp"

const std::array<std::array<int, 2>, 14> MP_SKELETON = {{
    {0, 9}, {1, 3}, {2, 4}, {3, 5},
    {4, 6}, {5, 7}, {6, 8}, {9, 10},
    {11, 13}, {12, 14}, {13, 15},
    {14, 16}, {17, 19}, {18, 20}
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