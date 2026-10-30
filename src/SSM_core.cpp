/*
░█▀▀░█▀▀░█▄█
░▀▀█░▀▀█░█░█
░▀▀▀░▀▀▀░▀░▀

SSM_core.cpp - Safe-Stop Monitor: robot capsule construction, braking-radii
computation and the SSM stop decision.
*/

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

#include "core.hpp"
#include "geometry.h"

Eigen::MatrixXd trajPoly5Numeric(
    double qi, double qi_p, double qi_pp,
    double qf, double qf_p, double qf_pp,
    double duration, int num_points) {
    // Same math as symbolic version but with plain doubles
    Eigen::VectorXd coeff(6);
    coeff(0) = qi;
    coeff(1) = qi_p;
    coeff(2) = qi_pp / 2.0;

    Eigen::Vector3d rhs;
    rhs(0) = qf    - qi    - qi_p  * duration - 0.5 * qi_pp * duration * duration;
    rhs(1) = qf_p  - qi_p  - qi_pp * duration;
    rhs(2) = qf_pp - qi_pp;

    Eigen::Matrix3d V;
    V(0,0) = std::pow(duration, 3); V(0,1) = std::pow(duration, 4); V(0,2) = std::pow(duration, 5);
    V(1,0) = 3*duration*duration;   V(1,1) = 4*std::pow(duration,3); V(1,2) = 5*std::pow(duration,4);
    V(2,0) = 6*duration;            V(2,1) = 12*duration*duration;   V(2,2) = 20*std::pow(duration,3);

    Eigen::Vector3d c345 = V.lu().solve(rhs);
    coeff(3) = c345(0);
    coeff(4) = c345(1);
    coeff(5) = c345(2);

    // Sample at num_points
    Eigen::MatrixXd traj(num_points, 3);
    for (int i = 0; i < num_points; ++i) {
        double t = i * duration / (double)(num_points - 1);
        traj(i, 0) = coeff(0) + coeff(1)*t + coeff(2)*t*t
                   + coeff(3)*t*t*t + coeff(4)*t*t*t*t + coeff(5)*t*t*t*t*t;
        traj(i, 1) = coeff(1) + 2*coeff(2)*t + 3*coeff(3)*t*t
                   + 4*coeff(4)*t*t*t + 5*coeff(5)*t*t*t*t;
        traj(i, 2) = 2*coeff(2) + 6*coeff(3)*t + 12*coeff(4)*t*t
                   + 20*coeff(5)*t*t*t;
    }
    return traj;
}


// ── computeRobotCapsules ──────────────────────────────────────────────
std::array<Capsule, 4> computeRobotCapsules(
    const RobotModel&      robot,
    const Eigen::VectorXd& q) {
    // One forward-kinematics pass, then read all frame poses from the cache
    robot.ComputeFK(q);

    // Mirrors compute_robot_capsules in Python
    // Capsule 1: link0 → link1
    Eigen::Vector3d A1 = robot.GetJointPose("panda_link1").translation();
    Eigen::Vector3d B1 = Eigen::Vector3d::Zero();  // base origin

    // Capsule 2: link2 → link3
    Eigen::Vector3d A2 = robot.GetJointPose("panda_link3").translation();
    Eigen::Vector3d B2 = robot.GetJointPose("panda_link2").translation();

    // Capsule 3: link4 → link5
    Eigen::Vector3d A3 = robot.GetJointPose("panda_link5").translation();
    Eigen::Vector3d B3 = robot.GetJointPose("panda_link4").translation();

    // Capsule 4: link6 → link7
    Eigen::Vector3d A4 = robot.GetJointPose("panda_link8").translation();
    Eigen::Vector3d B4 = robot.GetJointPose("panda_link7").translation();

    return {Capsule{A1, B1, RV[0]}, Capsule{A2, B2, RV[1]},
            Capsule{A3, B3, RV[2]}, Capsule{A4, B4, RV[3]}};
}


// ── computeMaxVelCapsule ──────────────────────────────────────────────
std::array<double, 4> computeMaxVelCapsule(
    const RobotModel&            robot,
    const Eigen::VectorXd&       q,
    const Eigen::VectorXd&       q_p,
    const std::array<Capsule, 4>& capsules,
    const Eigen::Vector3d&       C_h,
    const Eigen::Vector3d&       C_r) {
    // Direction vector from robot to human contact point
    Eigen::Vector3d C      = C_h - C_r;
    double          C_norm = C.norm();

    std::array<double, 4> vel_max;

    // Frame IDs for each capsule endpoint
    // Mirrors the DH-based compute in Python, using Pinocchio frames instead
    const std::array<std::string, 4> frame_A = {
        "panda_link1", "panda_link3", "panda_link5", "panda_link7"};
    const std::array<std::string, 4> frame_B = {
        "", "panda_link2", "panda_link4", "panda_link6"};  // "" = world origin

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d A = capsules[i].A;
        Eigen::Vector3d B = capsules[i].B;

        // Get Jacobian at endpoint A
        const Eigen::MatrixXd J = robot.ComputeJacobian(frame_A[i], q);

        Eigen::Vector3d A_p     = J.topRows(3) * q_p;     // linear velocity of A
        Eigen::Vector3d omega   = J.bottomRows(3) * q_p;  // angular velocity

        // Velocity of capsule surface endpoints in direction of C
        // Mirrors: A_ep = A_p + cross(omega, (A-B)/|A-B| * rv)
        Eigen::Vector3d AB = A - B;
        double AB_norm = AB.norm();

        Eigen::Vector3d A_ep = A_p + omega.cross(AB / AB_norm * capsules[i].radius);
        Eigen::Vector3d B_ep = A_p + omega.cross(
            (B - A) / AB_norm * (AB_norm + capsules[i].radius));

        double vel_A = A_ep.dot(C) / C_norm;
        double vel_B = B_ep.dot(C) / C_norm;

        vel_max[i] = std::max({vel_A, vel_B, 0.0});
    }

    return vel_max;
}


// capsuleCalculation
std::array<double, 4> capsuleCalculation(
    double                  T_stop,
    const Eigen::VectorXd&  q,
    const Eigen::VectorXd&  q_p,
    const Eigen::VectorXd&  q_pp,
    const Eigen::Vector3d&  C_h,
    const Eigen::Vector3d&  C_r,
    const RobotModel&       robot) {
    // Human safety distance during stop
    double Sh = MAX_HUMAN_SPEED * (T_REACTION + T_stop);

    // Sample braking trajectory
    int num_points = static_cast<int>(T_stop * 1000.0 / 5.0) + 1;  // every 5ms

    // Build braking trajectory for all 7 joints
    Eigen::MatrixXd Q(num_points, 7);
    Eigen::MatrixXd Q_p(num_points, 7);
    Eigen::MatrixXd Q_pp(num_points, 7);

    for (int j = 0; j < 7; j++) {
        Eigen::MatrixXd traj = trajPoly5Numeric(
            q(j), q_p(j), q_pp(j),
            q(j), 0.0,    0.0,
            T_stop, num_points);
        Q.col(j)    = traj.col(0);
        Q_p.col(j)  = traj.col(1);
        Q_pp.col(j) = traj.col(2);
    }

    // Compute max capsule velocity at each trajectory point
    std::array<std::vector<double>, 4> vel_max_traj;

    for (int m = 0; m < num_points; m++) {
        Eigen::VectorXd q_m   = Q.row(m).transpose();
        Eigen::VectorXd qp_m  = Q_p.row(m).transpose();

        auto capsules = computeRobotCapsules(robot, q_m);
        auto vel_max  = computeMaxVelCapsule(robot, q_m, qp_m,
                                              capsules, C_h, C_r);
        for (int i = 0; i < 4; ++i)
            vel_max_traj[i].push_back(vel_max[i]);
    }

    // Time vector for integration
    Eigen::VectorXd dt_stop = Eigen::VectorXd::LinSpaced(num_points, 0.0, T_stop);

    // Compute r_sw for each capsule
    // r_sw = rv + Sh + Sr + Ss + csi
    std::array<double, 4> r_sw;
    for (int i = 0; i < 4; i++) {
        double Sr = vel_max_traj[i][0] * T_REACTION;         // reaction distance
        double Ss = trapz(dt_stop, vel_max_traj[i]);         // braking distance
        r_sw[i]   = RV[i] + Sh + Sr + Ss + CSI;

        /*std::cout << "capsule " << i << ": T_stop=" << T_stop
                  << " RV=" << RV[i] << " Sh=" << Sh
                  << " Sr=" << Sr << " Ss=" << Ss
                  << " vel0=" << vel_max_traj[i][0]
                  << " vel_max=" << *std::max_element(vel_max_traj[i].begin(), vel_max_traj[i].end())
                  << " r_sw=" << r_sw[i] << "\n";*/
    }

    return r_sw;
}


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
    const DistanceResult&   closest) {
    
    if (skeleton.empty()) return false;

    // The radii are logged by the collision checker on every call, so they
    // must hold a defined value even when no valid pair is found below
    radii.fill(0.0);

    // Compute robot capsules at current config
    auto robot_capsules = computeRobotCapsules(robot, q);

    // Compute robot safety radii along braking trajectory
    auto r_sw_r = capsuleCalculation(T_stop, q, q_p, q_pp,
                                      closest.c_h, closest.c_r, robot);
                                      
    //std::cout << "saving radius\n";
    
    radii[0] = r_sw_r[0]; radii[1] = r_sw_r[1]; radii[2] = r_sw_r[2]; radii[3] = r_sw_r[3];

    // Stop condition: distance < human_radius + robot_radius
    bool exitflag = closest.length < R_SW_H[closest.ind_h] + r_sw_r[closest.ind_r];

    /*std::cout << "Min distance: " << min_dist
              << " threshold: " << R_SW_H[ind_h] + r_sw_r[ind_r] << "\n";*/

    return exitflag;
}
