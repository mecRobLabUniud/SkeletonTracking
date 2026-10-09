#pragma once


#include <Eigen/Dense>
#include <vector>
#include <string>

class RobotModel;


// ── Quintic polynomial trajectory between two points (zero vel/acc at the
//    ends), equivalent to quinticpolytraj(...) with 2 waypoints ─────────────
struct Traj {
    Eigen::MatrixXd p, v, a;  // dim x n
};

Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1,
                     double T, double dt, int n);


// ── Trajectory container ───────────────────────────────────────────────────
struct Trajectory {
    std::vector<Eigen::VectorXd> q;
    std::vector<Eigen::VectorXd> qd;
    std::vector<Eigen::VectorXd> qdd;
    std::vector<Eigen::Vector3d> p;
    std::vector<Eigen::Vector3d> pd;
    std::vector<double> t;
};


// ── Point-to-point trajectory from CSV waypoints + IK ──────────────────────
// Samples every segment at `dt` for N samples per segment.
Trajectory load_p2p_trajectory(RobotModel& robot, const std::string& c_dir,
                               int n_traj, int N, double dt,
                               const std::string& p_csv = "p_ref.csv",
                               const std::string& t_csv = "t_ref.csv");
