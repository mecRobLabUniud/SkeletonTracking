/*
skeleton.hpp - Canonical skeleton point container shared by the tracking, safety
and RULA code.
*/

#pragma once
#include <vector>

#include <Eigen/Core>

// 15 joints or more, each a 3D point
using Skeleton = std::vector<Eigen::Vector3d>;
