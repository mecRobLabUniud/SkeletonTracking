#pragma once

#include <Eigen/Dense>

#include "robot_model.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Kinematic and dynamic joint limits for the Franka Panda arm
// ─────────────────────────────────────────────────────────────────────────────
struct KinematicsLimits {
    KinematicsLimits();

    Eigen::MatrixXd q_limits;
    Eigen::MatrixXd qd_limits;
    Eigen::MatrixXd qdd_limits;
};

// ─────────────────────────────────────────────────────────────────────────────
// Output of one SSM+PFL QP step
// ─────────────────────────────────────────────────────────────────────────────
struct SSMPFLResult {
    Eigen::VectorXd qdd_next;
    Eigen::VectorXd qd_next;
    Eigen::VectorXd q_next;
    Eigen::Vector3d p_next;
    Eigen::Vector3d pd_next;
    bool exitflag;
};

// ─────────────────────────────────────────────────────────────────────────────
// Solve one SSM+PFL QP step for joint acceleration subject to joint limits
// and per-link safety constraints against an obstacle at ro
// ─────────────────────────────────────────────────────────────────────────────
SSMPFLResult SSMPFL(const RobotModel& robot,
                     double dt,
                     double stopping_time,
                     const Eigen::VectorXd& q_t,
                     const Eigen::VectorXd& qdot_t,
                     const Eigen::Vector3d& x_ref,
                     const Eigen::Vector3d& xd_ref,
                     const Eigen::VectorXd& q_ref,
                     Eigen::Vector3d ro,
                     const Eigen::Vector3d& vo,
                     double delta,
                     double Qpj, 
                     double Qpt, 
                     double Qv);
