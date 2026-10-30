/*
░█▀▀░█▀█░█▀▄░█▀▀
░█░░░█░█░█▀▄░█▀▀
░▀▀▀░▀▀▀░▀░▀░▀▀▀

core.hpp - Public interface of the SSM+PFL layer: joint-limit tables, the QP result struct and the SSMPFL / SSMPFLescape solvers.
*/

#pragma once

#include "robot_model.hpp"
#include <array>
#include <vector>
#include <Eigen/Dense>
#include "shared_state.h"
#include "min_distance_calculation.hpp"

//  Types

// A capsule is defined by two 3D endpoints
struct Capsule {
    Eigen::Vector3d A;  // first endpoint
    Eigen::Vector3d B;  // second endpoint
    double          radius;
};

//  Constants
// Robot capsule radii
const std::array<double, 4> RV         = {0.085, 0.085, 0.06, 0.065};
// Human segment safety radii
// Skeleton segment connectivity indices into the 21-point merged skeleton
// (utils.data_transmitter "MERGED" topic, reshape_structure() in
// data_merging.py). Joint index meaning, per web_interface.py's docstring:
//   0 head             7 left hand        14 right knee
//   1 left shoulder    8 right hand       15 left ankle
//   2 right shoulder   9 upper torso      16 right ankle
//   3 left elbow       10 lower torso     17 left heel
//   4 right elbow      11 left hip        18 right heel
//   5 left wrist       12 right hip       19 left foot
//   6 right wrist      13 left knee       20 right foot
const std::array<double, 14> R_SW_H    = {0.16, 0.06, 0.06, 0.06, 0.06, 0.1, 0.1,
                                          0.15, 0.1, 0.1, 0.08, 0.08, 0.05, 0.05};

const double T_REACTION      = 0.05;   // seconds
const double CSI             = 0.0;    // Zd + Zr constant
const double MAX_HUMAN_SPEED = 1.6;    // m/s
const double Qpj = 70.0;
const double Qpt = 1.0;
const double Qv = 0.08;


// ─────────────────────────────────────────────────────────────────────────────
// Non-symbolic 5th degree polynomial trajectory (mirrors TrajPoly5 in Python)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::MatrixXd trajPoly5Numeric(
    double qi, double qi_p, double qi_pp,
    double qf, double qf_p, double qf_pp,
    double duration, int num_points);

// ─────────────────────────────────────────────────────────────────────────────
// Compute the 4 robot capsule endpoints from joint config using RobotModel FK
// ─────────────────────────────────────────────────────────────────────────────
std::array<Capsule, 4> computeRobotCapsules(
    const RobotModel&      robot,
    const Eigen::VectorXd& q);

// ─────────────────────────────────────────────────────────────────────────────
// Compute max velocity of capsule endpoints toward human
// ─────────────────────────────────────────────────────────────────────────────
std::array<double, 4> computeMaxVelCapsule(
    const RobotModel&           robot,
    const Eigen::VectorXd&      q,
    const Eigen::VectorXd&      q_p,
    const std::array<Capsule, 4>& capsules,
    const Eigen::Vector3d&      C_h,
    const Eigen::Vector3d&      C_r);

// ─────────────────────────────────────────────────────────────────────────────
// Compute robot safety radii for each capsule along braking trajectory
// ─────────────────────────────────────────────────────────────────────────────
std::array<double, 4> capsuleCalculation(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Eigen::Vector3d&  C_h,
    const Eigen::Vector3d&  C_r,
    const RobotModel&       robot);

// ─────────────────────────────────────────────────────────────────────────────
// Kinematic and dynamic joint limits for the Franka Panda arm
// ─────────────────────────────────────────────────────────────────────────────
struct KinematicsLimits {
    KinematicsLimits() {
        q_limits.resize(2, 7);
        q_limits << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                     2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973;

        qd_limits.resize(2, 7);
        qd_limits << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100,
                      2.1750,  2.1750,  2.1750,  2.1750,  2.6100,  2.6100,  2.6100;

        qdd_limits.resize(2, 7);
        qdd_limits << -15, -7.5, -10, -12.5, -15, -20, -20,
                       15,   7.5,  10,  12.5,  15,  20,  20;
    }

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
// Solve SSM calculation for separation distance subject to joint limits
// and per-link safety constraints against an obstacle at ro
// ─────────────────────────────────────────────────────────────────────────────
bool SSM(double T_stop,
        const Eigen::VectorXd&  q,
        const Eigen::VectorXd&  q_p,
        const Eigen::VectorXd&  q_pp,
        const Skeleton&         skeleton,
        const RobotModel&       robot,
        std::array<double, 4>&  radii,
        const DistanceResult&   closest);


// ─────────────────────────────────────────────────────────────────────────────
// Solve SSM+PFL calculation for joint acceleration subject to joint limits
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