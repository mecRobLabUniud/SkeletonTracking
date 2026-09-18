#pragma once


#include <Eigen/Dense>
#include <vector>
#include <array>
#include <string>


// ── CSV I/O ────────────────────────────────────────────────────────────────
std::vector<std::array<double, 7>> load_trajectory_CSV(const std::string& path);


void save_trajectory_CSV(const std::string& path,
                         const std::vector<std::array<double, 7>>& traj);


std::vector<double> load_timestamps_CSV(const std::string& path);


void save_timestamps_CSV(const std::string& path,
                         const std::vector<double>& times);


// ── Binary I/O ────────────────────────────────────────────────────────────
std::vector<std::array<double, 7>> load_bin(const std::string& path);


void save_bin(const std::string& path,
              const std::vector<std::array<double, 7>>& traj);


// ── Finite-difference derivative estimation ─────────────────────────────────
Eigen::VectorXd estimate_velocities(const Eigen::VectorXd& t,
                                    const Eigen::VectorXd& q);


Eigen::VectorXd estimate_accelerations(const Eigen::VectorXd& t,
                                       const Eigen::VectorXd& q);


// ── Quintic Hermite basis functions ──────────────────────────────────────────
inline double quintic_hermite(double s, double h,
                               double q0, double v0, double a0,
                               double q1, double v1, double a1) {
    double s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
    double H0 = 1 - 10*s3 + 15*s4 - 6*s5;
    double H1 = s - 6*s3 + 8*s4 - 3*s5;
    double H2 = 0.5*s2 - 1.5*s3 + 1.5*s4 - 0.5*s5;
    double H3 = 10*s3 - 15*s4 + 6*s5;
    double H4 = -4*s3 + 7*s4 - 3*s5;
    double H5 = 0.5*s3 - s4 + 0.5*s5;
    return q0*H0 + v0*h*H1 + a0*h*h*H2 + q1*H3 + v1*h*H4 + a1*h*h*H5;
}

inline double quintic_hermite_vel(double s, double h,
                                   double q0, double v0, double a0,
                                   double q1, double v1, double a1) {
    double s2=s*s, s3=s2*s, s4=s3*s;
    double dH0 = -30*s2 + 60*s3 - 30*s4;
    double dH1 = 1 - 18*s2 + 32*s3 - 15*s4;
    double dH2 = s - 4.5*s2 + 6*s3 - 2.5*s4;
    double dH3 = 30*s2 - 60*s3 + 30*s4;
    double dH4 = -12*s2 + 28*s3 - 15*s4;
    double dH5 = 1.5*s2 - 4*s3 + 2.5*s4;
    double dp_ds = q0*dH0 + v0*h*dH1 + a0*h*h*dH2 + q1*dH3 + v1*h*dH4 + a1*h*h*dH5;
    return dp_ds / h;   // chain rule: d/dt = (1/h) d/ds
}

inline double quintic_hermite_acc(double s, double h,
                                   double q0, double v0, double a0,
                                   double q1, double v1, double a1) {
    double s2=s*s, s3=s2*s;
    double d2H0 = -60*s + 180*s2 - 120*s3;
    double d2H1 = -36*s + 96*s2 - 60*s3;
    double d2H2 = 1 - 9*s + 18*s2 - 10*s3;
    double d2H3 = 60*s - 180*s2 + 120*s3;
    double d2H4 = -24*s + 84*s2 - 60*s3;
    double d2H5 = 3*s - 12*s2 + 10*s3;
    double d2p_ds2 = q0*d2H0 + v0*h*d2H1 + a0*h*h*d2H2 + q1*d2H3 + v1*h*d2H4 + a1*h*h*d2H5;
    return d2p_ds2 / (h*h);  // d²/dt² = (1/h²) d²/ds²
}


// ── Per-joint quintic spline interpolation ──────────────────────────────────
void quintic_spline_interp_full(const Eigen::VectorXd& t_low,
                                 const Eigen::VectorXd& q_low,
                                 const Eigen::VectorXd& t_high,
                                 Eigen::VectorXd& q_high,
                                 Eigen::VectorXd& v_high,
                                 Eigen::VectorXd& a_high);


// ── Main interpolation entry point ─────────────────────────────────────────
struct Trajectory {
    std::vector<Eigen::VectorXd> q;       // position
    std::vector<Eigen::VectorXd> qd;      // velocity
    std::vector<Eigen::VectorXd> qdd;     // acceleration
};

Trajectory interpolate_to_1kHz_full(
        const std::vector<std::array<double, 7>>& traj_low,
        std::vector<double> time_low);


// ── Sanity checks ────────────────────────────────────────────────────────
void validate_trajectory(const std::vector<std::array<double, 7>>& traj,
                         double rate_hz = 1000.0);