/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░█░▀█▀░▀█▀░█░░░█▀▀
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░█░░█░░░█░░█░░░▀▀█
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀▀▀░▀▀▀

trajectory_utils.hpp - Declaration of the quintic trajectory generator and of the point-to-point trajectory loader.
*/

#pragma once


#include <Eigen/Dense>
#include <optional>
#include <vector>
#include <string>

class RobotModel;

// ─────────────────────────────────────────────────────────────────────────────
// Quintic polynomial trajectory between two points (zero velocity and
// acceleration at both ends)
// ─────────────────────────────────────────────────────────────────────────────
struct Traj {
    Eigen::MatrixXd p, v, a;
};

// ─────────────────────────────────────────────────────────────────────────────
// Time-indexed joint and Cartesian trajectory container
// ─────────────────────────────────────────────────────────────────────────────
struct Trajectory {
    std::vector<Eigen::VectorXd> q;
    std::vector<Eigen::VectorXd> qd;
    std::vector<Eigen::VectorXd> qdd;
    std::vector<Eigen::Vector3d> p;
    std::vector<Eigen::Vector3d> pd;
    std::vector<double> t;
};

// ─────────────────────────────────────────────────────────────────────────────
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n);

// ─────────────────────────────────────────────────────────────────────────────
// Append one quintic segment to the output trajectory
// ─────────────────────────────────────────────────────────────────────────────
void build_traj(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                        std::vector<double>* vnorm, Trajectory& out, const double t_end, 
                        double& t, RobotModel& robot, double dt);





double computeAddedTime(const Eigen::VectorXd& q_end, const Eigen::VectorXd& q_c);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir,
                               int n_traj, double dt);

// ─────────────────────────────────────────────────────────────────────────────
// Build a point-to-point trajectory from the waypoints of a trajectory set
// ─────────────────────────────────────────────────────────────────────────────
Trajectory restart_p2p_trajectory(RobotModel& robot, const Eigen::VectorXd& q_restart,
                                    const double& t_restart, double dt);
