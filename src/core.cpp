#include <cmath>
#include <qpOASES.hpp>
#include <iostream>

#include "core.hpp"
#include "minDistance.hpp"


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

// ─────────────────────────────────────────────────────────────────────────────
// Solve one SSM+PFL QP step for joint acceleration subject to joint limits
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

    const Eigen::Matrix<double, 10, 1> residual = A*qddot_t - b;
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
    qp.setOptions(options);

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
