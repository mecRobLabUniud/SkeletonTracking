/*
░█▀▀░█▀█░█░░░█▀▀░░░░░█▀▀░▀█▀░█▀█░█▀█░░░░░█▀▄░█░█░█▀▄░█▀█░▀█▀░▀█▀░█▀█░█▀█
░█░░░█▀█░█░░░█░░░░░░░▀▀█░░█░░█░█░█▀▀░░░░░█░█░█░█░█▀▄░█▀█░░█░░░█░░█░█░█░█
░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░░░▀▀▀░▀▀░░▀▀▀░▀░▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀

calc_stop_duration.h - Declaration of the CasADi stop-duration optimiser (NLP solved with IPOPT) returning the minimum braking time.
*/

#pragma once
#include <casadi/casadi.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <Eigen/Dense>
#include "robot_model.hpp"

struct Poly5Traj {
    casadi::SX q;    // (N+1) x 1
    casadi::SX q_p;
    casadi::SX q_pp;
};

class StopDurationOptimizer
{
public:
    StopDurationOptimizer(const RobotModel& robot);
    StopDurationOptimizer(const pinocchio::Model& model);

    double solve(const Eigen::VectorXd& q,
                 const Eigen::VectorXd& q_p,
                 const Eigen::VectorXd& q_pp);

private:
    // Solver (created once in constructor)
    casadi::Function solver_;
    double stop_duration_prev_;

    // Constants
    static constexpr double LBX  = 0.05;
    static constexpr double UBX  = 0.4;
    static constexpr double P1   = 1e6;
    static constexpr double P2   = 1e5;
    static constexpr int    N    = 10;
    static constexpr double TOLL = 0.9;

    // Joint limits (Panda)
    casadi::SX q_min_lim_;
    casadi::SX q_max_lim_;
    casadi::SX q_p_lim_;
    casadi::SX q_pp_lim_;
    casadi::SX q_ppp_lim_;
    casadi::SX tau_lim_;
    casadi::SX tau_p_lim_;

    // Pinocchio model cast to CasADi scalar type for symbolic dynamics
    pinocchio::ModelTpl<casadi::SX> cmodel_;

    // Internal build methods
    Poly5Traj trajPoly5(casadi::SX qi, casadi::SX qi_p, casadi::SX qi_pp,
                          casadi::SX qf, casadi::SX qf_p, casadi::SX qf_pp,
                          casadi::SX x);

    casadi::SX  buildConstraints(casadi::SX x, casadi::SX p);
    casadi::Function createSolver();
};