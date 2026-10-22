/*
░█▀▀░█▀█░█▀▄░█▀▀
░█░░░█░█░█▀▄░█▀▀
░▀▀▀░▀▀▀░▀░▀░▀▀▀

core.hpp - Public interface of the SSM+PFL layer: joint-limit tables, the QP result struct and the SSMPFL / SSMPFLescape solvers.
*/

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

const double Qpj = 70.0;
const double Qpt = 1.0;
const double Qv = 0.08;

// ─────────────────────────────────────────────────────────────────────────────
// Solve one SSM+PFL QP step for joint acceleration subject to joint limits
// and per-link safety constraints against an obstacle at ro
// ─────────────────────────────────────────────────────────────────────────────
bool SSMPFL(const RobotModel& robot,
                    double dt,
                    double stopping_time,
                    const Eigen::VectorXd& q_ref,
                    const Eigen::VectorXd& qdot_ref,
                    const Eigen::VectorXd& qddot_ref,
                    const std::vector<Eigen::Vector3d>& ro_vec,
                    const std::vector<Eigen::Vector3d>& vo_vec,
                    const std::vector<double>& delta_vec);

// ─────────────────────────────────────────────────────────────────────────────
// Solve one SSM+PFL+escape QP step for joint acceleration subject to joint limits
// and per-link safety constraints against an obstacle at ro
// ─────────────────────────────────────────────────────────────────────────────
SSMPFLResult SSMPFLescape(const RobotModel& robot,
                        double dt,
                        double stopping_time,
                        const Eigen::VectorXd& q_t,
                        const Eigen::VectorXd& qdot_t,
                        const Eigen::Vector3d& x_ref,
                        const Eigen::Vector3d& xd_ref,
                        const Eigen::VectorXd& q_ref,
                        const std::vector<Eigen::Vector3d>& ro_vec,
                        const std::vector<Eigen::Vector3d>& vo_vec,
                        const std::vector<double>& delta_vec);