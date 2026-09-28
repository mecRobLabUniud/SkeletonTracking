#include "SSMPFL.hpp"

#include <cmath>
#include <iostream>

#include <qpOASES.hpp>

#include "minDistance.hpp"



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

// OptimizationWeights::OptimizationWeights() {
//     Qpj = 1.0;
//     Qpt = 1.0;
//     Qv = 1.0;
// }


// Single shared instance of the joint limits — built once instead of on
// every call to SSMPFL().
static const KinematicsLimits k_limits;
// static const OptimizationWeights weights;

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
                     double Qv) {
    const int n = 7;


    Qpj = Qpj;
    Qpt = Qpt;
    Qv = Qv;

    // std::cout << "Qpj =" << Qpj << std::endl;
    // std::cout << "Qpt =" << Qpt << std::endl;
    // std::cout << "Qv =" << Qv << std::endl;

    Eigen::VectorXd q_tp = q_t + dt * qdot_t;

    Eigen::Vector3d x_t = robot.GetJointPose("panda_link8", q_tp).translation();
    Eigen::MatrixXd J_t = robot.ComputeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::MatrixXd Jd_t = robot.ComputeDerivativeJacobian("panda_link8", q_t).bottomRows(3);

    ro = ro + vo * dt;

    // --- Objective: joint-space + task-space tracking ------------------
    Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(7, 7);
    weight_matrix(0, 0) = 1.5;
    weight_matrix(1, 1) = 3.0;
    weight_matrix(2, 2) = 3.0;
    weight_matrix(3, 3) = 1.75;
    weight_matrix(4, 4) = 1.75;
    weight_matrix(5, 5) = 0.5;
    weight_matrix(6, 6) = 0.1;

    // weight_matrix(0, 0) = 1;
    // weight_matrix(1, 1) = 1;
    // weight_matrix(2, 2) = 1;
    // weight_matrix(3, 3) = 1;
    // weight_matrix(4, 4) = 1;
    // weight_matrix(5, 5) = 1;
    // weight_matrix(6, 6) = 1;

    double dt2 = dt * dt;
    double dt4 = dt2 * dt2;

    Eigen::MatrixXd Hq = dt4/4.0*weight_matrix;
    Eigen::MatrixXd Hx = dt4/2.0*J_t.transpose()*J_t;
    Eigen::MatrixXd Hv = dt2*2.0*J_t.transpose()*J_t;

    Eigen::VectorXd fq = dt2/2.0*(q_t + dt*qdot_t - q_ref);
    Eigen::VectorXd fx = dt2*J_t.transpose()*(x_t + dt*J_t*qdot_t  + dt2/2.0*Jd_t*qdot_t - x_ref);
    Eigen::VectorXd fv = dt*2.0*J_t.transpose()*(J_t*qdot_t + dt*Jd_t*qdot_t - xd_ref);

    Eigen::MatrixXd H = Qpj*Hq + Qpt*Hx + Qv*Hv;
    Eigen::VectorXd f = Qpj*fq + Qpt*fx + Qv*fv;

    // --- Kinematic / dynamic bounds ------------------------------------
    Eigen::VectorXd qmin = (k_limits.q_limits.transpose().col(0) - q_t - dt*qdot_t)*2.0/dt2;
    Eigen::VectorXd qmax = (k_limits.q_limits.transpose().col(1) - q_t - dt*qdot_t)*2.0/dt2;
    Eigen::VectorXd qdmin = (k_limits.qd_limits.transpose().col(0) - qdot_t)/dt;
    Eigen::VectorXd qdmax = (k_limits.qd_limits.transpose().col(1) - qdot_t)/dt;
    Eigen::VectorXd qddmin = k_limits.qdd_limits.transpose().col(0);
    Eigen::VectorXd qddmax = k_limits.qdd_limits.transpose().col(1);

    Eigen::VectorXd q_lb = qmin.cwiseMax(qdmin).cwiseMax(qddmin);
    Eigen::VectorXd q_ub = qmax.cwiseMin(qdmax).cwiseMin(qddmax);

    // --- SSM+PFL constraints (per-link kinematics) ----------------------
    Eigen::MatrixXd J1 = robot.ComputeJacobian("panda_link2", q_t).bottomRows(3);
    Eigen::MatrixXd J1d = robot.ComputeDerivativeJacobian("panda_link2", q_t).bottomRows(3);
    Eigen::Vector3d r1 = robot.GetJointPose("panda_link2", q_t).translation();

    Eigen::MatrixXd J2 = robot.ComputeJacobian("panda_link3", q_t).bottomRows(3);
    Eigen::MatrixXd J2d = robot.ComputeDerivativeJacobian("panda_link3", q_t).bottomRows(3);
    Eigen::Vector3d r2 = robot.GetJointPose("panda_link3", q_t).translation();

    Eigen::MatrixXd J3 = robot.ComputeJacobian("panda_link4", q_t).bottomRows(3);
    Eigen::MatrixXd J3d = robot.ComputeDerivativeJacobian("panda_link4", q_t).bottomRows(3);
    Eigen::Vector3d r3 = robot.GetJointPose("panda_link4", q_t).translation();

    Eigen::MatrixXd J4 = robot.ComputeJacobian("panda_link5", q_t).bottomRows(3);
    Eigen::MatrixXd J4d = robot.ComputeDerivativeJacobian("panda_link5", q_t).bottomRows(3);
    Eigen::Vector3d r4 = robot.GetJointPose("panda_link5", q_t).translation();

    Eigen::MatrixXd J5 = robot.ComputeJacobian("panda_link7", q_t).bottomRows(3);
    Eigen::MatrixXd J5d = robot.ComputeDerivativeJacobian("panda_link7", q_t).bottomRows(3);
    Eigen::Vector3d r5 = robot.GetJointPose("panda_link7", q_t).translation();

    Eigen::MatrixXd J6 = robot.ComputeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::MatrixXd J6d = robot.ComputeDerivativeJacobian("panda_link8", q_t).bottomRows(3);
    Eigen::Vector3d r6 = robot.GetJointPose("panda_link8", q_t).translation();

    Eigen::MatrixXd A(10, n);
    Eigen::VectorXd b(10);


    A.row(0) = (ro.transpose()*J5 - r5.transpose()*J5)*dt;
    A.row(1) = (ro.transpose()*J6 - r5.transpose()*J6 - (r6 - r5).transpose()*J5)*dt;
    A.row(2) = (ro.transpose()*J1 - r1.transpose()*J1)*dt;
    A.row(3) = (ro.transpose()*J2 - r1.transpose()*J2 - (r2 - r1).transpose()*J1)*dt;
    A.row(4) = (ro.transpose()*J2 - r2.transpose()*J2)*dt;
    A.row(5) = (ro.transpose()*J3 - r2.transpose()*J3 - (r3 - r2).transpose()*J2)*dt;
    A.row(6) = (ro.transpose()*J3 - r3.transpose()*J3)*dt;
    A.row(7) = (ro.transpose()*J4 - r3.transpose()*J4 - (r4 - r3).transpose()*J3)*dt;
    A.row(8) = (ro.transpose()*J4 - r4.transpose()*J4)*dt;
    A.row(9) = (ro.transpose()*J5 - r4.transpose()*J5 - (r5 - r4).transpose()*J4)*dt;
    



    double d2 = delta * delta;

    // std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
    // std::cout << "ro = " << ro << std::endl;
    // std::cout << "vo = " << vo << std::endl;
    // std::cout << "ro.transpose()*J5 - r5.transpose()*J5 = " << ro.transpose()*J5 - r5.transpose()*J5 << std::endl;
    // std::cout << "(ro - r5).transpose()*J5d = " << (ro - r5).transpose()*J5d << std::endl;

    
    b(0) = 1.0/stopping_time*(std::pow(minsSSM(r5, r6, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J5 - r5.transpose()*J5)*qdot_t).value()
           - (dt*(ro - r5).transpose()*J5d*qdot_t).value();

    b(1) = 1.0/stopping_time*(std::pow(minsSSM(r5, r6, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J6 - r5.transpose()*J6 - (r6 - r5).transpose()*J5)*qdot_t).value()
           - (dt*(ro - r6).transpose()*J6d*qdot_t).value();

    b(2) = 1.0/stopping_time*(std::pow(minsSSM(r1, r2, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J1 - r1.transpose()*J1)*qdot_t).value()
           - (dt*(ro - r1).transpose()*J1d*qdot_t).value();

    b(3) = 1.0/stopping_time*(std::pow(minsSSM(r1, r2, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J2 - r1.transpose()*J2 - (r2 - r1).transpose()*J1)*qdot_t).value()
           - (dt*(ro - r2).transpose()*J2d*qdot_t).value();

    b(4) = 1.0/stopping_time*(std::pow(minsSSM(r2, r3, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J2 - r2.transpose()*J2)*qdot_t).value()
           - (dt*(ro - r2).transpose()*J2d*qdot_t).value();

    b(5) = 1.0/stopping_time*(std::pow(minsSSM(r2, r3, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J3 - r2.transpose()*J3 - (r3 - r2).transpose()*J2)*qdot_t).value()
           - (dt*(ro - r3).transpose()*J3d*qdot_t).value();

    b(6) = 1.0/stopping_time*(std::pow(minsSSM(r3, r4, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J3 - r3.transpose()*J3)*qdot_t).value()
           - (dt*(ro - r3).transpose()*J3d*qdot_t).value();

    b(7) = 1.0/stopping_time*(std::pow(minsSSM(r3, r4, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J4 - r3.transpose()*J4 - (r4 - r3).transpose()*J3)*qdot_t).value()
           - (dt*(ro - r4).transpose()*J4d*qdot_t).value();

    b(8) = 1.0/stopping_time*(std::pow(minsSSM(r4, r5, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J4 - r4.transpose()*J4)*qdot_t).value()
           - (dt*(ro - r4).transpose()*J4d*qdot_t).value();

    b(9) = 1.0/stopping_time*(std::pow(minsSSM(r4, r5, ro, delta), 2) - d2/4.0)
           - ((ro.transpose()*J5 - r4.transpose()*J5 - (r5 - r4).transpose()*J4)*qdot_t).value()
           - (dt*(ro - r5).transpose()*J5d*qdot_t).value();

    

    int nV = H.rows();
    int nC = A.rows();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_rm = H;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_rm = A;

    qpOASES::QProblem qp(nV, nC);

    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    options.terminationTolerance = 1e-6;
    qp.setOptions(options);

    int nWSR = 30;

    Eigen::VectorXd lbA = Eigen::VectorXd::Constant(nC, -qpOASES::INFTY);
    Eigen::VectorXd ubA = Eigen::VectorXd::Constant(nC, qpOASES::INFTY);

    qpOASES::returnValue status = qp.init(H_rm.data(), f.data(), A_rm.data(),
                                           q_lb.data(), q_ub.data(),
                                           lbA.data(), b.data(),
                                           nWSR);

    Eigen::VectorXd qddot(nV);
    qp.getPrimalSolution(qddot.data());

    bool success = (status == qpOASES::SUCCESSFUL_RETURN);
    int simpleStatus = qpOASES::getSimpleStatus(status);

    if (!success) {
        std::cout << "QP failed to solve. Status: " << simpleStatus << std::endl;
    }

    SSMPFLResult out;
    out.exitflag = success;

    std::cout << "+++++++++++++++++++++++++++++" << std::endl;
    
    std::cout << "Hp --------------\n" << Qpj*Hq + Qpt*Hx  << std::endl;
    std::cout << "Hv --------------\n" << Hv << std::endl;
    std::cout << "fp --------------\n" << Qpj*fq + Qpt*fx  << std::endl;
    std::cout << "fv --------------\n" << fv << std::endl;
    std::cout << "H --------------\n" << H << std::endl;
    std::cout << "f --------------\n" << f << std::endl;
    std::cout << "A --------------\n" << A << std::endl;
    std::cout << "b --------------\n" << b << std::endl;

    std::cout << "qddot --------------\n" << qddot << std::endl;
    std::cout << "A*qddot --------------\n" << A*qddot << std::endl;


    if (out.exitflag) {
        out.qdd_next = qddot;
        out.q_next = q_t + dt * qdot_t + dt2 / 2.0 * out.qdd_next;
        out.qd_next = qdot_t + dt * out.qdd_next;

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