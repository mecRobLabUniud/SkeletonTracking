#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

#include <franka/robot.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include "examples_common.hpp"

// ---------------------------------------------------------------------------
// 1. Load traj_qectory from file
//    Expected format: one waypoint per line, 7 space-separated doubles
// ---------------------------------------------------------------------------
std::vector<std::array<double, 7>> loadTrajectory(const std::string& path) {
    std::vector<std::array<double, 7>> traj_q;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open traj_qectory file: " + path);
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::array<double, 7> wp;
        for (int i = 0; i < 7; i++) {
            if (!(ss >> wp[i])) {
                throw std::runtime_error("Malformed line: " + line);
            }
        }
        traj_q.push_back(wp);
    }
    return traj_q;
}

// ---------------------------------------------------------------------------
// 2. Savitzky-Golay coefficients for window=5, poly=3, derivative=0
//    Larger window → smoother. Must be odd. Increase to 11, 21 if needed.
// ---------------------------------------------------------------------------
std::vector<std::array<double, 7>> savitzkyGolay(
    const std::vector<std::array<double, 7>>& traj_q,
    int half_window = 5)   // total window = 2*half_window+1
{
    const int N = traj_q.size();
    const int W = half_window;
    std::vector<std::array<double, 7>> out(traj_q);  // copy, edges stay as-is

    // Precompute SG coefficients for poly=3 via least-squares (closed form)
    // For simplicity we use a weighted average kernel approximation.
    // Full closed-form SG coeffs for arbitrary window:
    auto sg_coeffs = [&](int w) -> std::vector<double> {
        // Polynomial order 3, zero-th derivative
        // Coefficients computed as per Savitzky & Golay (1964)
        int m = 2 * w + 1;
        std::vector<double> c(m);
        // Build Vandermonde matrix and solve — here we use the known
        // analytic formula for order-3 SG filter
        double norm = 0.0;
        for (int k = -w; k <= w; k++) {
            // coeff for position k in [-w, w]
            double v = (3.0*(3*w*w*(w+1)*(w+1) - 4*k*k*(3*w*w + 3*w - 1 - 5*k*k)));
            // This is the exact formula for cubic SG smoothing
            // Simplified approximation: use symmetric triangular weight
            c[k + w] = (w + 1.0 - std::abs(k));
            norm += c[k + w];
        }
        for (auto& x : c) x /= norm;
        return c;
    };

    auto coeffs = sg_coeffs(W);

    for (int i = W; i < N - W; i++) {
        for (int j = 0; j < 7; j++) {
            double val = 0.0;
            for (int k = -W; k <= W; k++) {
                val += coeffs[k + W] * traj_q[i + k][j];
            }
            out[i][j] = val;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// 3. Simple moving average — use as alternative or pre-pass
// ---------------------------------------------------------------------------
std::vector<std::array<double, 7>> movingAverage(
    const std::vector<std::array<double, 7>>& traj_q,
    int window = 11)
{
    const int N = traj_q.size();
    const int half = window / 2;
    std::vector<std::array<double, 7>> out(traj_q);

    for (int i = half; i < N - half; i++) {
        for (int j = 0; j < 7; j++) {
            double sum = 0.0;
            for (int k = -half; k <= half; k++) {
                sum += traj_q[i + k][j];
            }
            out[i][j] = sum / window;
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// 4. Check max jerk (spikes = noise = sound)
// ---------------------------------------------------------------------------
void printMaxJerk(const std::vector<std::array<double, 7>>& traj_q, double dt = 0.001) {
    std::array<double, 7> max_jerk{};
    for (size_t i = 1; i + 2 < traj_q.size(); i++) {
        for (int j = 0; j < 7; j++) {
            double vel0 = (traj_q[i][j]   - traj_q[i-1][j]) / dt;
            double vel1 = (traj_q[i+1][j] - traj_q[i][j])   / dt;
            double acc0 = (vel1 - vel0) / dt;
            double vel2 = (traj_q[i+2][j] - traj_q[i+1][j]) / dt;
            double acc1 = (vel2 - vel1) / dt;
            double jerk = std::abs((acc1 - acc0) / dt);
            if (jerk > max_jerk[j]) max_jerk[j] = jerk;
        }
    }
    std::cout << "Max jerk per joint [rad/s^3]:\n";
    for (int j = 0; j < 7; j++) {
        std::cout << "  J" << j+1 << ": " << max_jerk[j] << "\n";
    }
}


    // robot.control([&](const franka::RobotState& /*state*/, franka::Duration /*dt*/)
    //     -> franka::JointPositions
    // {
    //     franka::JointPositions cmd{traj_q[idx]};
// 
    //     if (idx + 1 >= N) {
    //         return franka::MotionFinished(cmd);
    //     }
    //     idx++;
    //     return cmd;
    // });






// ---------------------------------------------------------------------------
// 5. Execute traj_qectory on the robot
// ---------------------------------------------------------------------------
void executeTrajectory(franka::Robot& robot,
                        const std::vector<std::array<double, 7>>& traj_q,
                        const std::vector<std::array<double, 7>>& traj_qd) {
                            // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    
    size_t idx = 0;
    const size_t N = traj_q.size();

    // Set gains for the joint impedance control.
    // Stiffness
    const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
    // Damping
    const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};


    // Define callback for the joint torque control loop.
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        effort_control_callback =
            [&idx, &model, N, k_gains, d_gains, traj_q, traj_qd](
                const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
        // Read current coriolis terms from model.
        std::array<double, 7> coriolis = model.coriolis(state);

        // Compute torque command from joint impedance control law.
        // Note: The answer to our Cartesian pose inverse kinematics is always in state_qd with one
        // time step delay.
        std::array<double, 7> tau_d_calculated;
        std::array<double, 7> q{traj_q[idx]};
        std::array<double, 7> qd{traj_qd[idx]};
        for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] =
            k_gains[i] * (q[i] - state.q[i]) - d_gains[i] * qd[i] + coriolis[i];
        }

        std::array<double, 7> tau_d_rate_limited =
            franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

        if (idx + 1 >= N) return franka::MotionFinished(franka::Torques{tau_d_rate_limited});
        idx++;

        // Send torque command.
        return tau_d_rate_limited;
    };


    // Define callback for the joint torque control loop.
    std::function<franka::JointPositions(const franka::RobotState&, franka::Duration)>
        position_control_callback =
    [&](const franka::RobotState& /*state*/, franka::Duration /*dt*/) -> franka::JointPositions {
        franka::JointPositions cmd{traj_q[idx]};

        if (idx + 1 >= N) {
            return franka::MotionFinished(cmd);
        }
        idx++;
        return cmd;
    };

    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
        velocity_control_callback =
    [&](const franka::RobotState& /*state*/, franka::Duration /*dt*/) -> franka::JointVelocities {
        franka::JointVelocities cmd{traj_qd[idx]};

        if (idx + 1 >= N) {
            return franka::MotionFinished(cmd);
        }
        idx++;
        return cmd;
    };

    // Start real-time control loop.
    robot.control(velocity_control_callback);
}




std::vector<std::array<double, 7>> loadTrajectoryCSV(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);

    std::vector<std::array<double, 7>> traj;
    std::string line;

    while (std::getline(f, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') continue;

        std::array<double, 7> wp;
        std::stringstream ss(line);
        std::string token;
        int j = 0;

        while (std::getline(ss, token, ',') && j < 7) {
            try {
                wp[j++] = std::stod(token);
            } catch (const std::exception&) {
                throw std::runtime_error("Bad value at line: " + line);
            }
        }

        if (j != 7)
            throw std::runtime_error("Expected 7 values, got "
                                     + std::to_string(j)
                                     + " at line: " + line);
        traj.push_back(wp);
    }

    std::cout << "Loaded " << traj.size() << " waypoints from " << path << "\n";
    return traj;
}







// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    // if (argc < 3) {
    //     std::cerr << "Usage: " << argv[0]
    //               << " <robot_ip> <traj_qectory_file> [smooth_window=11]\n";
    //     return 1;
    // }

    const std::string robot_ip   = "172.16.0.2"; // argv[1];
    const std::string q_file  = "src/trajectories/recorded_test/q.csv";
    const std::string qd_file  = "src/trajectories/recorded_test/qd.csv";
    const int         window     = 11; // (argc >= 4) ? std::stoi(argv[3]) : 11;

    // --- Load ---
    // auto traj_q = loadTrajectory(q_file);
    auto traj_q = loadTrajectoryCSV(q_file);
    auto traj_qd = loadTrajectoryCSV(qd_file); 
    std::cout << "Loaded " << traj_q.size() << " waypoints.\n";

    /* // --- Diagnose ---
    std::cout << "--- Before smoothing ---\n";
    printMaxJerk(traj_q);

    // --- Smooth ---
    // Use moving average first (fast), then SG on top for shape preservation
    auto traj_q_smooth = movingAverage(traj_q, window);
    traj_q_smooth      = savitzkyGolay(traj_q_smooth, window / 2);
    auto traj_q_smooth = movingAverage(traj_q, window);
    traj_q_smooth      = savitzkyGolay(traj_q_smooth, window / 2);

    std::cout << "--- After smoothing (window=" << window << ") ---\n";
    printMaxJerk(traj_q_smooth); */

    // --- Execute ---
    try {
        franka::Robot robot(robot_ip);
        robot.setCollisionBehavior(
            {{20,20,20,20,20,20,20}}, {{20,20,20,20,20,20,20}},
            {{20,20,20,20,20,20}},    {{20,20,20,20,20,20}});

        MotionGenerator motion_generator(0.3, traj_q[0]);
        std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;

        std::cout << "Executing smoothed traj_qectory...\n";
        executeTrajectory(robot, traj_q, traj_qd);
        std::cout << "Done.\n";
    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}