/*
░▀█▀░█▀▄░█▀█░▀▀█░█▀▀░█▀▀░▀█▀░█▀█░█▀▄░█░█░░░░░█░█░▀█▀░▀█▀░█░░░█▀▀
░░█░░█▀▄░█▀█░░░█░█▀▀░█░░░░█░░█░█░█▀▄░░█░░░░░░█░█░░█░░░█░░█░░░▀▀█
░░▀░░▀░▀░▀░▀░▀▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░░▀░░▀▀▀░▀▀▀░▀▀▀

trajectory_utils.hpp - Declaration of the quintic trajectory generator and of the point-to-point trajectory loader.
*/

#pragma once


#include <Eigen/Dense>
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
// Sample a quintic polynomial trajectory between x0 and x1
// ─────────────────────────────────────────────────────────────────────────────
Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n);

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
// Build a point-to-point trajectory from the waypoints of a trajectory set
//
// The waypoints are read from <c_dir>/src/trajectories/test<n_traj>/ and can
// be given either in Cartesian coordinates (p_ref.csv, solved with the inverse
// kinematics) or in joint coordinates (q_ref.csv, used as they are). The
// cumulative time of each waypoint is read from t_ref.csv and gives the
// duration of each segment.
//
// p_csv / t_csv override the file names; when they are left empty the format
// is detected from the files that are present in the trajectory directory.
//
// N is the number of samples used for a segment whose duration is unknown
// (a segment with a non-positive duration carries no motion and is skipped).
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir,
                               int n_traj, double dt,
                               const std::string& p_csv = "",
                               const std::string& t_csv = "");
