#pragma once

#include <Eigen/Dense>

#include "robot_model.hpp"

// Kinematic/dynamic joint limits for the Franka Panda arm.
//
// NOTE: previously this was written as a struct whose members were
// initialized inline with constructor-style syntax
// (e.g. `Eigen::MatrixXd q_limits(2, 7);`) and then accessed via
// `KinematicsLimits.q_limits` as though `KinematicsLimits` were an
// object rather than a type. That does not compile. This version gives
// the struct a real constructor and is meant to be instantiated once
// (see the `static const KinematicsLimits` instance in SSMPFL.cpp).
struct KinematicsLimits {
    KinematicsLimits();

    Eigen::MatrixXd q_limits;    // 2x7: row 0 = min, row 1 = max
    Eigen::MatrixXd qd_limits;   // 2x7
    Eigen::MatrixXd qdd_limits;  // 2x7
};

struct SSMPFLResult {
    Eigen::VectorXd qdd_next;
    Eigen::VectorXd qd_next;
    Eigen::VectorXd q_next;
    Eigen::Vector3d p_next;
    Eigen::Vector3d pd_next;
    bool exitflag;
};

// Solves one step of the SSM+PFL QP (Eq. 2.70) for joint acceleration,
// subject to kinematic/dynamic joint limits and per-link speed-and-
// separation-monitoring safety constraints against an obstacle at `ro`.
SSMPFLResult SSMPFL(const RobotModel& robot,
                     double delta_t,
                     double stopping_time,
                     const Eigen::VectorXd& q_t,
                     const Eigen::VectorXd& qdot_t,
                     const Eigen::Vector3d& x_ref_tplusone,
                     const Eigen::Vector3d& xd_ref_tplusone,
                     Eigen::Vector3d ro,
                     const Eigen::Vector3d& vo,
                     double delta,
                     const Eigen::VectorXd& q_des,
                     double Qv);
