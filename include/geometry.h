#pragma once
#include <vector>
#include <array>
#include <optional>
#include <Eigen/Dense>
#include "flag_stop.h"  // for Skeleton, DistanceResult, SKEL_INDEX

//  Math utilities 

// Clamp n to [0, 1]
double clamp(double n);

// Trapezoidal integration
double trapz(const Eigen::VectorXd&        dt,
             const std::vector<double>&    values);

//  Distance functions 

// Minimum distance between two 3D line segments
// P1-Q1: human segment, P2-Q2: robot capsule axis
// Returns distance and closest points on each segment
DistanceResult distanceToSegment(
    const Eigen::Vector3d& P1, const Eigen::Vector3d& Q1,
    const Eigen::Vector3d& P2, const Eigen::Vector3d& Q2);

// Minimum distance from a robot capsule axis (P2-Q2) to any skeleton segment
// Returns the distance result and the index of the closest human segment
std::pair<std::optional<DistanceResult>, int> distanceToSkeleton(
    const Skeleton&        skeleton,
    const Eigen::Vector3d& P2,
    const Eigen::Vector3d& Q2);