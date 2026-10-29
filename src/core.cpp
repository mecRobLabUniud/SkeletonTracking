/*
░█▀▀░█▀█░█▀▄░█▀▀
░█░░░█░█░█▀▄░█▀▀
░▀▀▀░▀▀▀░▀░▀░▀▀▀

core.cpp - Core of the SSM+PFL layer: joint-limit and potential-field obstacle constraints for the Panda, with the QP step solved by qpOASES.
*/

#include <cmath>
#include <qpOASES.hpp>
#include <iostream>

#include "core.hpp"
#include "minDistance.hpp"
#include "geometry.h"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <iostream>
#include <algorithm>
#include <limits>
#include <cmath>


// ─────────────────────────────────────────────────────────────────────────────
// Initialize the Franka Panda joint position, velocity and acceleration limits
// ─────────────────────────────────────────────────────────────────────────────
KinematicsLimits::KinematicsLimits() {
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

// ── Shared joint-limit instance ─────────────────────────────────────────────
static const KinematicsLimits k_limits;



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
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    const Eigen::VectorXd&  q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    // Mirrors compute_robot_capsules in Python
    // Capsule 1: link0 → link1
    Eigen::Vector3d A1 = data.oMf[model.getFrameId("panda_link1")].translation();
    Eigen::Vector3d B1 = Eigen::Vector3d::Zero();  // base origin

    // Capsule 2: link2 → link3
    Eigen::Vector3d A2 = data.oMf[model.getFrameId("panda_link3")].translation();
    Eigen::Vector3d B2 = data.oMf[model.getFrameId("panda_link2")].translation();

    // Capsule 3: link4 → link5
    Eigen::Vector3d A3 = data.oMf[model.getFrameId("panda_link5")].translation();
    Eigen::Vector3d B3 = data.oMf[model.getFrameId("panda_link4")].translation();

    // Capsule 4: link6 → link7
    Eigen::Vector3d A4 = data.oMf[model.getFrameId("panda_link7")].translation();
    Eigen::Vector3d B4 = data.oMf[model.getFrameId("panda_link6")].translation();

    return {Capsule{A1, B1, RV[0]}, Capsule{A2, B2, RV[1]},
            Capsule{A3, B3, RV[2]}, Capsule{A4, B4, RV[3]}};
}


// ── computeMaxVelCapsule ──────────────────────────────────────────────
std::array<double, 4> computeMaxVelCapsule(
    const pinocchio::Model&       model,
    pinocchio::Data&              data,
    const Eigen::VectorXd&        q,
    const Eigen::VectorXd&        q_p,
    const std::array<Capsule, 4>& capsules,
    const Eigen::Vector3d&        C_h,
    const Eigen::Vector3d&        C_r) {
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
        Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, model.nq);
        J.setZero();
        pinocchio::computeFrameJacobian(model, data, q,
            model.getFrameId(frame_A[i]),
            pinocchio::LOCAL_WORLD_ALIGNED, J);

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
    const pinocchio::Model& model,
    pinocchio::Data&        data) {
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

        auto capsules = computeRobotCapsules(model, data, q_m);
        auto vel_max  = computeMaxVelCapsule(model, data, q_m, qp_m,
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
    const pinocchio::Model& model,
    pinocchio::Data&        data,
    std::array<double, 4>&  radii,
    NewDistanceResult*         out_closest) {
    
    if (skeleton.empty()) return false;

    // The radii are logged by the collision checker on every call, so they
    // must hold a defined value even when no valid pair is found below
    radii.fill(0.0);

    // Compute robot capsules at current config
    auto robot_capsules = computeRobotCapsules(model, data, q);

    // Find minimum distance between any robot capsule and any skeleton segment
    double          min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d min_C_h = Eigen::Vector3d::Constant(NAN);
    Eigen::Vector3d min_C_r = Eigen::Vector3d::Constant(NAN);
    int             ind_h = 0, ind_r = 0;
    bool            found = false;

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d P2 = robot_capsules[i].A;
        Eigen::Vector3d Q2 = robot_capsules[i].B;
        // also radius??

        auto [dist, ind_h_temp] = distanceToSkeleton(skeleton, P2, Q2);
        //std::cout << "dist=" << dist->distance << "\n";

        if (!dist.has_value()) continue;    // skip iteration if dist is null
        if (!found || dist->distance < min_dist) {
            min_dist = dist->distance;
            min_C_h  = dist->C_h;
            min_C_r  = dist->C_r;
            ind_h    = ind_h_temp;
            ind_r    = i;
            found    = true;
        }
    }
    //std::cout << "Found: " << found << "\n";

    if (out_closest) {
        // Report the outcome even when no valid pair was found: NaN is the
        // "no data" signal downstream (the collision checker publishes it as
        // JSON null and the retry logic skips the obstacle wait).
        out_closest->distance = found ? min_dist : NAN;
        out_closest->C_h      = min_C_h;
        out_closest->C_r      = min_C_r;
    }

    if (!found) return false;

    // Compute robot safety radii along braking trajectory
    auto r_sw_r = capsuleCalculation(T_stop, q, q_p, q_pp,
                                      min_C_h, min_C_r, model, data);
                                      
    //std::cout << "saving radius\n";
    
    radii[0] = r_sw_r[0]; radii[1] = r_sw_r[1]; radii[2] = r_sw_r[2]; radii[3] = r_sw_r[3];

    // Stop condition: distance < human_radius + robot_radius
    bool should_stop = min_dist < R_SW_H[ind_h] + r_sw_r[ind_r];

    /*std::cout << "Min distance: " << min_dist
              << " threshold: " << R_SW_H[ind_h] + r_sw_r[ind_r] << "\n";*/

    return should_stop;
}


// ─────────────────────────────────────────────────────────────────────────────
// Solve SSM+PFL calculation for joint acceleration subject to joint limits
// and per-link safety constraints against an obstacle at ro
// ─────────────────────────────────────────────────────────────────────────────
bool SSMPFL(const RobotModel& robot,
                    double dt,
                    double stopping_time,
                    const Eigen::VectorXd& q_t,
                    const Eigen::VectorXd& qdot_t,
                    const Eigen::VectorXd& qddot_t,
                    const std::vector<Eigen::Vector3d>& ro_vec,
                    const std::vector<Eigen::Vector3d>& vo_vec,
                    const std::vector<double>& delta_vec) {
    const int n = 7;
    double dt2 = dt * dt;

    // ── Current and predicted task state ────────────────────────────────────
    Eigen::VectorXd q_tp = q_t + dt * qdot_t;

    Eigen::Vector3d x_t = robot.GetJointPose("panda_link8", q_tp).translation();
    Eigen::MatrixXd J_t = robot.ComputeJacobian("panda_link8", q_t).topRows(3);
    Eigen::MatrixXd Jd_t = robot.ComputeDerivativeJacobian("panda_link8", q_t).topRows(3);

    // ── Kinematic and dynamic bounds ────────────────────────────────────────
    Eigen::VectorXd qmin = (k_limits.q_limits.transpose().col(0) - q_t - dt * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qmax = (k_limits.q_limits.transpose().col(1) - q_t - dt * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qdmin = (k_limits.qd_limits.transpose().col(0) - qdot_t) / dt;
    Eigen::VectorXd qdmax = (k_limits.qd_limits.transpose().col(1) - qdot_t) / dt;
    Eigen::VectorXd qddmin = k_limits.qdd_limits.transpose().col(0);
    Eigen::VectorXd qddmax = k_limits.qdd_limits.transpose().col(1);

    Eigen::VectorXd q_lb = qmin.cwiseMax(qdmin).cwiseMax(qddmin);
    Eigen::VectorXd q_ub = qmax.cwiseMin(qdmax).cwiseMin(qddmax);

    // ── SSM+PFL constraints, per-link kinematics ────────────────────────────
    const char* names[6] = {"panda_link2","panda_link3","panda_link4",
                            "panda_link5","panda_link7","panda_link8"};
    Eigen::MatrixXd J[6], Jd[6];
    Eigen::Vector3d r[6];
    for (int i = 0; i < 6; ++i) {
        J[i]  = robot.ComputeJacobian(names[i], q_t).topRows(3);
        Jd[i] = robot.ComputeDerivativeJacobian(names[i], q_t).topRows(3);
        r[i]  = robot.GetJointPose(names[i], q_t).translation();
    }

    // ── Stack constraints over all obstacles ──
    const int N  = static_cast<int>(ro_vec.size());
    const int nC = 10 * N;
    Eigen::MatrixXd A(nC, n);
    Eigen::VectorXd b(nC);

    int row = 0;
    for (int j = 0; j < ro_vec.size(); j++) {
        const Eigen::Vector3d ro = ro_vec[j] + vo_vec[j] * dt;   // per-obstacle prediction
        double delta = delta_vec[j];
        const double d2 = delta * delta;

        for (int k = 0; k < 5; ++k) {        // segment (link k, link k+1)
            const int a = k, c = k + 1;
            const double s    = minsSSM(r[a], r[c], ro, delta);
            const double base = 10.0 / stopping_time * (s * s - d2 / 4.0);

            // row for endpoint a
            Eigen::RowVectorXd Aa = (ro - r[a]).transpose() * J[a];
            A.row(row) = Aa * dt;
            b(row) = base - (Aa * qdot_t).value()
                        - (dt * (ro - r[a]).transpose() * Jd[a] * qdot_t).value();
            ++row;

            // row for endpoint c
            Eigen::RowVectorXd Ac = ro.transpose() * J[c] - r[a].transpose() * J[c]
                                - (r[c] - r[a]).transpose() * J[a];
            A.row(row) = Ac * dt;
            b(row) = base - (Ac * qdot_t).value()
                        - (dt * (ro - r[c]).transpose() * Jd[c] * qdot_t).value();
            ++row;
        }
    }

    // auto check_matrix = A*qddot_t - b;
    // bool success = (check_matrix.array() <= 0).all();

    // The residual holds one value per constraint row, i.e. 10 per obstacle:
    // it is dynamic, so that it fits both a single obstacle and no obstacle at
    // all (an empty residual trivially satisfies every constraint).
    const Eigen::VectorXd residual = A*qddot_t - b;
    bool success = (residual.array() <= 0.0).all();

    return success;
}


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
                        const std::vector<double>& delta_vec) {
    const int n = 7;
    double dt2 = dt * dt;
    double dt4 = dt2 * dt2;

    // ── Current and predicted task state ────────────────────────────────────
    Eigen::VectorXd q_tp = q_t + dt * qdot_t;

    Eigen::Vector3d x_t = robot.GetJointPose("panda_link8", q_tp).translation();
    Eigen::MatrixXd J_t = robot.ComputeJacobian("panda_link8", q_t).topRows(3);
    Eigen::MatrixXd Jd_t = robot.ComputeDerivativeJacobian("panda_link8", q_t).topRows(3);

    // ── Objective: joint-space and task-space tracking ──────────────────────
    Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(7, 7);
    weight_matrix(0, 0) = 1.5;
    weight_matrix(1, 1) = 3.0;
    weight_matrix(2, 2) = 3.0;
    weight_matrix(3, 3) = 1.75;
    weight_matrix(4, 4) = 1.75;
    weight_matrix(5, 5) = 1.0;
    weight_matrix(6, 6) = 0.1;

    Eigen::MatrixXd Hq = dt4/4.0*weight_matrix;
    Eigen::MatrixXd Hx = dt4/2.0*J_t.transpose()*J_t;
    Eigen::MatrixXd Hv = dt2*2.0*J_t.transpose()*J_t;

    Eigen::VectorXd fq = dt2/2.0*(q_t + dt*qdot_t - q_ref);
    Eigen::VectorXd fx = dt2*J_t.transpose()*(x_t + dt*J_t*qdot_t  + dt2/2.0*Jd_t*qdot_t - x_ref);
    Eigen::VectorXd fv = dt*2.0*J_t.transpose()*(J_t*qdot_t + dt*Jd_t*qdot_t - xd_ref);

    Eigen::MatrixXd H = Qpj*Hq + Qpt*Hx + Qv*Hv;
    Eigen::VectorXd f = Qpj*fq + Qpt*fx + Qv*fv;

    // ── Kinematic and dynamic bounds ────────────────────────────────────────
    Eigen::VectorXd qmin = (k_limits.q_limits.transpose().col(0) - q_t - dt * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qmax = (k_limits.q_limits.transpose().col(1) - q_t - dt * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qdmin = (k_limits.qd_limits.transpose().col(0) - qdot_t) / dt;
    Eigen::VectorXd qdmax = (k_limits.qd_limits.transpose().col(1) - qdot_t) / dt;
    Eigen::VectorXd qddmin = k_limits.qdd_limits.transpose().col(0);
    Eigen::VectorXd qddmax = k_limits.qdd_limits.transpose().col(1);

    Eigen::VectorXd q_lb = qmin.cwiseMax(qdmin).cwiseMax(qddmin);
    Eigen::VectorXd q_ub = qmax.cwiseMin(qdmax).cwiseMin(qddmax);

    // ── SSM+PFL constraints, per-link kinematics ────────────────────────────
    const char* names[6] = {"panda_link2","panda_link3","panda_link4",
                            "panda_link5","panda_link7","panda_link8"};
    Eigen::MatrixXd J[6], Jd[6];
    Eigen::Vector3d r[6];
    for (int i = 0; i < 6; ++i) {
        J[i]  = robot.ComputeJacobian(names[i], q_t).topRows(3);
        Jd[i] = robot.ComputeDerivativeJacobian(names[i], q_t).topRows(3);
        r[i]  = robot.GetJointPose(names[i], q_t).translation();
    }

    // ── Stack constraints over all obstacles ──
    const int N  = static_cast<int>(ro_vec.size());
    const int nC = 10 * N;
    Eigen::MatrixXd A(nC, n);
    Eigen::VectorXd b(nC);

    int row = 0;
    for (int j = 0; j < ro_vec.size(); j++) {
        const Eigen::Vector3d ro = ro_vec[j] + vo_vec[j] * dt;   // per-obstacle prediction
        double delta = delta_vec[j];
        const double d2 = delta * delta;

        for (int k = 0; k < 5; ++k) {        // segment (link k, link k+1)
            const int a = k, c = k + 1;
            const double s    = minsSSM(r[a], r[c], ro, delta);
            const double base = 10.0 / stopping_time * (s * s - d2 / 4.0);

            // row for endpoint a
            Eigen::RowVectorXd Aa = (ro - r[a]).transpose() * J[a];
            A.row(row) = Aa * dt;
            b(row) = base - (Aa * qdot_t).value()
                        - (dt * (ro - r[a]).transpose() * Jd[a] * qdot_t).value();
            ++row;

            // row for endpoint c
            Eigen::RowVectorXd Ac = ro.transpose() * J[c] - r[a].transpose() * J[c]
                                - (r[c] - r[a]).transpose() * J[a];
            A.row(row) = Ac * dt;
            b(row) = base - (Ac * qdot_t).value()
                        - (dt * (ro - r[c]).transpose() * Jd[c] * qdot_t).value();
            ++row;
        }
    }

    // ── QP setup and solve ──────────────────────────────────────────────────
    int nV = H.rows();
    // int nC = A.rows();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_rm = H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rm = A;

    qpOASES::QProblem qp(nV, nC);

    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    options.terminationTolerance = 1e-6;
    qp.setOptions(options);      // modifica

    int nWSR = 100;

    Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nC, -qpOASES::INFTY);

    qpOASES::returnValue status = qp.init(H_rm.data(), f.data(), A_rm.data(),
                                           q_lb.data(), q_ub.data(),
                                           lbA.data(), b.data(),
                                           nWSR);

    Eigen::VectorXd qddot(nV);
    qp.getPrimalSolution(qddot.data());

    bool success = (status == qpOASES::SUCCESSFUL_RETURN);

    // std::cout << "status: " << status << std::endl;

    // ── Build the result ────────────────────────────────────────────────────
    SSMPFLResult out;
    out.exitflag = success;

    if (out.exitflag) {
        out.q_next = q_t + dt * qdot_t + dt2 / 2.0 * qddot;
        out.qd_next = qdot_t + dt * qddot;
        out.qdd_next = qddot;
        out.p_next = robot.GetJointPose("panda_link8", out.q_next).translation();
        out.pd_next = robot.ComputeJacobian("panda_link8", out.q_next).topRows(3) * out.qd_next;
    } else {
        out.q_next = q_t;
        out.qd_next = Eigen::VectorXd::Zero(n);
        out.qdd_next = Eigen::VectorXd::Zero(n);
        out.p_next = x_t;
        out.pd_next = Eigen::Vector3d::Zero();
    }
    
    return out;
}
