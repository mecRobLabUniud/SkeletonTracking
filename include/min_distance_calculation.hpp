/*
░█▄█░▀█▀░█▀█░░░░░█▀▄░▀█▀░█▀▀░▀█▀░█▀█░█▀█░█▀▀░█▀▀░░░░░█▀▀░█▀█░█░░░█▀▀░█░█░█░░░█▀█░▀█▀░▀█▀░█▀█░█▀█
░█░█░░█░░█░█░░░░░█░█░░█░░▀▀█░░█░░█▀█░█░█░█░░░█▀▀░░░░░█░░░█▀█░█░░░█░░░█░█░█░░░█▀█░░█░░░█░░█░█░█░█
░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀░▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀

min_distance_calculation.hpp - Human-to-robot minimum distance routines and the
merged-skeleton bone list.
*/

#pragma once
#include <array>
#include <optional>

#include <Eigen/Core>

#include "robot_model.hpp"
#include "skeleton.hpp"

// Connectivity of the 21-point merged skeleton
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
    int ind_r;
};

DistanceResult segm_to_segm_distance(
    const Eigen::Vector3d& P1,
    const Eigen::Vector3d& Q1,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2);

std::optional<DistanceResult> human_to_segm_distance(
    const Skeleton& skeleton,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2,
    const int ind_r);

std::optional<DistanceResult> human_to_robot_distance(
    const Skeleton& skeleton,
    const RobotModel& robot,
    const Eigen::VectorXd& q);

// Clearance from a safety cylinder of radius delta/2 around segment [ra, rb]
// to the obstacle point ro.
double minsSSM(const Eigen::Vector3d& ra,
               const Eigen::Vector3d& rb,
               const Eigen::Vector3d& ro,
               double delta);
