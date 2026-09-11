#pragma once
#include "robot_model.hpp"
#include "minDistance.hpp"
#include <qpOASES.hpp>
       
struct SSMPFLResult {
    Eigen::VectorXd qdd_next;
    Eigen::VectorXd qd_next;
    Eigen::VectorXd q_next;
    Eigen::Vector3d p_next;
    Eigen::Vector3d pd_next;
    bool exitflag;
};

inline SSMPFLResult SSMPFL(const RobotModel& robot,
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
                            double Qv) {

    const int n = 7;
    Eigen::MatrixXd q_limits(2, 7);
    q_limits << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    Eigen::MatrixXd qd_limits(2, 7);
    qd_limits << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100,
                    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;
    Eigen::MatrixXd qdd_limits(2, 7);
    qdd_limits << -15, -7.5, -10, -12.5, -15, -20, -20,
                    15, 7.5, 10, 12.5, 15, 20, 20;

    Eigen::VectorXd q_tp = q_t + delta_t * qdot_t;

    Eigen::Vector3d x_t = robot.GetJointPose("panda_link8", q_tp).translation();
    Eigen::MatrixXd J_t = robot.ComputeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::MatrixXd Jd_t = robot.ComputeDerivativeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::Vector3d xd_t = J_t * qdot_t;
    (void)xd_t; // computed in source, unused there too

    ro = ro + vo * delta_t;

    // --- Objective: joint-space + task-space tracking ------------------
    Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(7, 7);
    weight_matrix(0, 0) = 1.5;
    weight_matrix(1, 1) = 3.0;
    weight_matrix(2, 2) = 3.0;
    weight_matrix(3, 3) = 1.75;
    weight_matrix(4, 4) = 1.75;
    weight_matrix(5, 5) = 0.1;
    weight_matrix(6, 6) = 0.1;
    // NOTE: weight_matrix is 6x6 in the source regardless of n; if your
    // robot doesn't have 6 joints, resize/adjust this block accordingly.

    double dt2 = delta_t * delta_t;
    double dt4 = dt2 * dt2;


    Eigen::MatrixXd Hp = 70.0 * dt4 / 4.0 * weight_matrix + 1.0 * dt4 / 2.0 * J_t.transpose() * J_t;

    Eigen::VectorXd kpp = (-x_ref_tplusone + dt2 / 2.0 * Jd_t * qdot_t + delta_t * J_t * qdot_t + x_t);
    Eigen::VectorXd fpp = dt2 * J_t.transpose() * kpp;

    Eigen::VectorXd kp = (-q_des + delta_t * qdot_t + q_t);
    Eigen::VectorXd fp = 70.0 * (dt2 / 2.0 * kp) + 1.0 * fpp;

    Eigen::MatrixXd Hv = dt2 * 2.0 * J_t.transpose() * J_t;
    Eigen::VectorXd kv = (xd_ref_tplusone - J_t * qdot_t - delta_t * Jd_t * qdot_t);
    Eigen::VectorXd fv = -delta_t * 2.0 * J_t.transpose() * kv;

    Eigen::MatrixXd H = Hp + Qv * Hv;
    Eigen::VectorXd f = fp + Qv * fv;

    


    // --- Kinematic / dynamic bounds ------------------------------------
    Eigen::VectorXd qmin = (q_limits.transpose().col(0) - q_t - delta_t * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qmax = (q_limits.transpose().col(1) - q_t - delta_t * qdot_t) * 2.0 / dt2;
    Eigen::VectorXd qdmin = (qd_limits.transpose().col(0) - qdot_t) / delta_t;
    Eigen::VectorXd qdmax = (qd_limits.transpose().col(1) - qdot_t) / delta_t;
    Eigen::VectorXd qddmin = qdd_limits.transpose().col(0);
    Eigen::VectorXd qddmax = qdd_limits.transpose().col(1);

    Eigen::VectorXd q_lb = qmin.cwiseMax(qdmin).cwiseMax(qddmin);
    Eigen::VectorXd q_ub = qmax.cwiseMin(qdmax).cwiseMin(qddmax);



    // robot.ComputeJacobian("panda_link8", q_t).bottomRows(3);

    // --- SSM+PFL constraints (per-link kinematics) ----------------------
    Eigen::MatrixXd J1  = robot.ComputeJacobian("panda_link2", q_t).bottomRows(3);
    Eigen::MatrixXd J1d = robot.ComputeDerivativeJacobian("panda_link2", q_t).bottomRows(3);
    Eigen::Vector3d r1  = robot.GetJointPose("panda_link2", q_t).translation();

    Eigen::MatrixXd J2  = robot.ComputeJacobian("panda_link3", q_t).bottomRows(3);
    Eigen::MatrixXd J2d = robot.ComputeDerivativeJacobian("panda_link3", q_t).bottomRows(3);
    Eigen::Vector3d r2  = robot.GetJointPose("panda_link3", q_t).translation();

    Eigen::MatrixXd J3  = robot.ComputeJacobian("panda_link4", q_t).bottomRows(3);
    Eigen::MatrixXd J3d = robot.ComputeDerivativeJacobian("panda_link4", q_t).bottomRows(3);
    Eigen::Vector3d r3  = robot.GetJointPose("panda_link4", q_t).translation();

    Eigen::MatrixXd J4  = robot.ComputeJacobian("panda_link5", q_t).bottomRows(3);
    Eigen::MatrixXd J4d = robot.ComputeDerivativeJacobian("panda_link5", q_t).bottomRows(3);
    Eigen::Vector3d r4  = robot.GetJointPose("panda_link5", q_t).translation();

    Eigen::MatrixXd J5  = robot.ComputeJacobian("panda_link7", q_t).bottomRows(3);
    Eigen::MatrixXd J5d = robot.ComputeDerivativeJacobian("panda_link7", q_t).bottomRows(3);
    Eigen::Vector3d r5  = robot.GetJointPose("panda_link7", q_t).translation();

    Eigen::MatrixXd J6  = robot.ComputeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::MatrixXd J6d = robot.ComputeDerivativeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::Vector3d r6  = robot.GetJointPose("panda_link8", q_t).translation();


    // NOTE: unlike SSMescape/SSMcheck, here the constraint rows are scaled
    // by delta_t only (not stopping_time), and the RHS margin term uses
    // 1/stopping_time instead — this matches Equation 2.70 in the source
    // (as opposed to Equation 2.52 used by SSMescape/SSMcheck).
    Eigen::MatrixXd A(10, n);
    Eigen::VectorXd b(10);

    A.row(0) = (ro.transpose() * J5 - r5.transpose() * J5) * delta_t;
    A.row(1) = (ro.transpose() * J6 - r5.transpose() * J6 - (r6 - r5).transpose() * J5) * delta_t;
    A.row(2) = (ro.transpose() * J1 - r1.transpose() * J1) * delta_t;
    A.row(3) = (ro.transpose() * J2 - r1.transpose() * J2 - (r2 - r1).transpose() * J1) * delta_t;
    A.row(4) = (ro.transpose() * J2 - r2.transpose() * J2) * delta_t;
    A.row(5) = (ro.transpose() * J3 - r2.transpose() * J3 - (r3 - r2).transpose() * J2) * delta_t;
    A.row(6) = (ro.transpose() * J3 - r3.transpose() * J3) * delta_t;
    A.row(7) = (ro.transpose() * J4 - r3.transpose() * J4 - (r4 - r3).transpose() * J3) * delta_t;
    A.row(8) = (ro.transpose() * J4 - r4.transpose() * J4) * delta_t;
    A.row(9) = (ro.transpose() * J5 - r4.transpose() * J5 - (r5 - r4).transpose() * J4) * delta_t;


    double d2 = delta * delta;

    b(0) = 1.0 / stopping_time * (std::pow(minsSSM(r5, r6, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J5 - r5.transpose() * J5) * qdot_t).value()
            - (delta_t * (ro - r5).transpose() * J5d * qdot_t).value();

    b(1) = 1.0 / stopping_time * (std::pow(minsSSM(r5, r6, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J6 - r5.transpose() * J6 - (r6 - r5).transpose() * J5) * qdot_t).value()
            - (delta_t * (ro - r6).transpose() * J6d * qdot_t).value();

    b(2) = 1.0 / stopping_time * (std::pow(minsSSM(r1, r2, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J1 - r1.transpose() * J1) * qdot_t).value()
            - (delta_t * (ro - r1).transpose() * J1d * qdot_t).value();

    b(3) = 1.0 / stopping_time * (std::pow(minsSSM(r1, r2, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J2 - r1.transpose() * J2 - (r2 - r1).transpose() * J1) * qdot_t).value()
            - (delta_t * (ro - r2).transpose() * J2d * qdot_t).value();

    b(4) = 1.0 / stopping_time * (std::pow(minsSSM(r2, r3, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J2 - r2.transpose() * J2) * qdot_t).value()
            - (delta_t * (ro - r2).transpose() * J2d * qdot_t).value();

    b(5) = 1.0 / stopping_time * (std::pow(minsSSM(r2, r3, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J3 - r2.transpose() * J3 - (r3 - r2).transpose() * J2) * qdot_t).value()
            - (delta_t * (ro - r3).transpose() * J3d * qdot_t).value();

    b(6) = 1.0 / stopping_time * (std::pow(minsSSM(r3, r4, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J3 - r3.transpose() * J3) * qdot_t).value()
            - (delta_t * (ro - r3).transpose() * J3d * qdot_t).value();

    b(7) = 1.0 / stopping_time * (std::pow(minsSSM(r3, r4, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J4 - r3.transpose() * J4 - (r4 - r3).transpose() * J3) * qdot_t).value()
            - (delta_t * (ro - r4).transpose() * J4d * qdot_t).value();

    b(8) = 1.0 / stopping_time * (std::pow(minsSSM(r4, r5, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J4 - r4.transpose() * J4) * qdot_t).value()
            - (delta_t * (ro - r4).transpose() * J4d * qdot_t).value();

    b(9) = 1.0 / stopping_time * (std::pow(minsSSM(r4, r5, ro, delta), 2) - d2 / 4.0)
            - ((ro.transpose() * J5 - r4.transpose() * J5 - (r5 - r4).transpose() * J4) * qdot_t).value()
            - (delta_t * (ro - r5).transpose() * J5d * qdot_t).value();




    // USING_NAMESPACE_QPOASES

    int nV = H.rows();       // was hardcoded to 6 — use actual size, not a magic number
    int nC = A.rows();       // number of inequality rows

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_rm = H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rm = A;

    qpOASES::QProblem qp(nV, nC);

    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    options.terminationTolerance = 1e-6;
    qp.setOptions(options);

    int nWSR = 1000000;

    // std::cout << "H: " << H << std::endl;
    // std::cout << "A: " << A << std::endl;
    // std::cout << "f: " << f << std::endl;
    // std::cout << "q_lb: " << q_lb << std::endl;
    // std::cout << "q_ub: " << q_ub << std::endl;
    // std::cout << "b: " << b << std::endl;

    Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nC, -qpOASES::INFTY);


    qpOASES::returnValue status = qp.init(H_rm.data(), f.data(), A_rm.data(),
                                        q_lb.data(), q_ub.data(),
                                        lbA.data(), b.data(),
                                        nWSR);

    Eigen::VectorXd qddot(nV);
    qp.getPrimalSolution(qddot.data());

    qpOASES::real_t fval = qp.getObjVal();

    // std::cout << "fval: " << fval << std::endl;
    std::cout << "qddot = " << qddot.transpose() << std::endl;

    bool success = (status == qpOASES::SUCCESSFUL_RETURN);
    int simpleStatus = qpOASES::getSimpleStatus(status);

    if (!success) {
        std::cout << "QP failed to solve. Status: " << simpleStatus << std::endl;
    }
                            

    SSMPFLResult out;
    out.exitflag = success;

    if (out.exitflag) {
        out.qdd_next = qddot;
        out.q_next = q_t + delta_t * qdot_t + dt2 / 2.0 * out.qdd_next;
        out.qd_next = qdot_t + delta_t * out.qdd_next;

        out.p_next = robot.GetJointPose("panda_link8", out.q_next).translation();

        Eigen::MatrixXd J_next = robot.ComputeJacobian("panda_link8", out.q_next).bottomRows(3);
        out.pd_next = J_next * out.qd_next;
    } else {
        out.qdd_next = Eigen::VectorXd::Zero(n);
        out.q_next = q_t;
        out.qd_next = Eigen::VectorXd::Zero(n);
        out.p_next = x_t;
        out.pd_next = Eigen::Vector3d::Zero();
    }


    return out;
}
