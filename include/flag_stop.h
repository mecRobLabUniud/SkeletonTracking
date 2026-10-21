#pragma once
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "shared_state.h"

//  Types

// A capsule is defined by two 3D endpoints
struct Capsule {
    Eigen::Vector3d A;  // first endpoint
    Eigen::Vector3d B;  // second endpoint
    double          radius;
};

// Result of a distance query
struct DistanceResult {
    double          distance;   // NaN when no valid pair could be computed
    Eigen::Vector3d C_h;  // closest point on human segment
    Eigen::Vector3d C_r;  // closest point on robot capsule
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
// Skeleton segment connectivity
const std::vector<std::array<int,2>> SKEL_INDEX = {
    {0, 9}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}, {6, 8}, 
    {9, 10}, {11, 13}, {12, 14}, {13, 15}, {14, 16}, {17, 19}, {18, 20}
};

const double T_REACTION      = 0.05;   // seconds
const double CSI             = 0.0;    // Zd + Zr constant
const double MAX_HUMAN_SPEED = 1.6;    // m/s

//  Function declarations 

// Non-symbolic 5th degree polynomial trajectory (mirrors TrajPoly5 in Python)
// Returns matrix of size [num_points x 3] where columns are [q, q_p, q_pp]
Eigen::MatrixXd trajPoly5Numeric(
    double qi, double qi_p, double qi_pp,
    double qf, double qf_p, double qf_pp,
    double duration, int num_points);

// Compute the 4 robot capsule endpoints from joint config using Pinocchio FK
std::array<Capsule, 4> computeRobotCapsules(
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    const Eigen::VectorXd&  q);

// Compute max velocity of capsule endpoints toward human
std::array<double, 4> computeMaxVelCapsule(
    const pinocchio::Model&    model,
    pinocchio::Data&           data,
    const Eigen::VectorXd&     q,
    const Eigen::VectorXd&     q_p,
    const std::array<Capsule, 4>& capsules,
    const Eigen::Vector3d&     C_h,
    const Eigen::Vector3d&     C_r);

// Compute robot safety radii for each capsule along braking trajectory
std::array<double, 4> capsuleCalculation(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Eigen::Vector3d&  C_h,
    const Eigen::Vector3d&  C_r,
    const pinocchio::Model& model,
    pinocchio::Data&        data);

// Main stop decision function
bool flagStop(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Skeleton&         skeleton,
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    std::array<double, 4>&  radii,
    DistanceResult*         out_closest = nullptr);