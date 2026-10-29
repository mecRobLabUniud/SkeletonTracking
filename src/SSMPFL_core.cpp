/*
░█▀▀░█▀▀░█▄█░█▀█░█▀▀░█░░
░▀▀█░▀▀█░█░█░█▀▀░█▀▀░█░░
░▀▀▀░▀▀▀░▀░▀░▀░░░▀░░░▀▀▀

SSMPFL_core.cpp - SSM+PFL feasibility check: joint-limit and per-link
obstacle constraints versus the commanded joint acceleration.
*/

#include <cmath>
#include <vector>

#include "core.hpp"
#include "min_distance_calculation.hpp"

// ── Joint-limit instance for this solver ────────────────────────────────────
static const KinematicsLimits k_limits;

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
