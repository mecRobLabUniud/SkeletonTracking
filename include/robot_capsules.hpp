/*
░ROBOT_CAPSULES░
░ROBOT_CAPSULES░
░ROBOT_CAPSULES░

robot_capsules.hpp
*/

#pragma once

/*
░█▀▄░█▀█░▀█▀░█▀█░░░▀█▀░█▀▄░█▀█░█▀█░█▀▀░█▄█░▀█▀░▀█▀░▀█▀░█▀▀░█▀▄
░█░█░█▀█░░█░░█▀█░░░░█░░█▀▄░█▀█░█░█░▀▀█░█░█░░█░░░█░░░█░░█▀▀░█▀▄
░▀▀░░▀░▀░░▀░░▀░▀░░░░▀░░▀░▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░░▀▀▀▀▀░▀░▀

Geometry of the robot capsules, as published to web_interface.py.

The robot is approximated by 4 capsules; their endpoints and radii are the
"x_robot" / "y_robot" / "z_robot" and "radius" fields of the update_plot
event (see scripts/flask_utils/js/capsules.js), while the joint positions
are published separately and used by the web interface to draw the robot
mesh (scripts/flask_utils/js/robot.js).
*/

#include <array>
#include <cstddef>

#include <Eigen/Dense>

#include "robot_model.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Robot capsule radii (mirrors RV in flag_stop.h)
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr std::array<double, 4> ROBOT_CAPSULE_RADII = {0.085, 0.085, 0.06, 0.065};

// ─────────────────────────────────────────────────────────────────────────────
// Endpoints of the 4 robot capsules, in the order expected by the web
// interface (ROBOT_CONFIG in capsules.js):
//
//   capsule 1: base origin -> panda_link1
//   capsule 2: panda_link2 -> panda_link3
//   capsule 3: panda_link4 -> panda_link5
//   capsule 4: panda_link6 -> panda_link7
//
// The first point is the base origin, so that the web interface can place the
// robot mesh at the origin of the world frame.
// ─────────────────────────────────────────────────────────────────────────────
inline std::array<std::array<double, 3>, 8> robotCapsulePoints(const RobotModel& robot,
                                                                const Eigen::VectorXd& q) {
    // One forward kinematics pass, then the frames are read from the cache
    robot.ComputeFK(q);

    const char* kLinkNames[7] = {"panda_link1", "panda_link2", "panda_link3",
                                 "panda_link4", "panda_link5", "panda_link6",
                                 "panda_link7"};

    Eigen::Vector3d p[7];
    for (int i = 0; i < 7; ++i) {
        p[i] = robot.GetJointPose(kLinkNames[i]).translation();
    }

    const Eigen::Vector3d base = Eigen::Vector3d::Zero();
    const Eigen::Vector3d pts[8] = {base, p[0], p[1], p[2], p[3], p[4], p[5], p[6]};

    std::array<std::array<double, 3>, 8> out{};
    for (size_t i = 0; i < 8; ++i) {
        out[i] = {pts[i].x(), pts[i].y(), pts[i].z()};
    }
    return out;
}
