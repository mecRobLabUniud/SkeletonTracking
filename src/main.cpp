#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <optional>
#include <charconv>
#include <cmath>

#include "trajectory_utils.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include "min_distance_calculation.hpp"
#include "robot_model.hpp"

// #include "COLLcheck.hpp"
#include "SSMPFL.hpp"

// ─────────────────────────────────────────────────────────────────────────────
bool do_once = true;
Eigen::VectorXd q_real;
Eigen::VectorXd qd_real;
Eigen::VectorXd qdd_real;
Eigen::Vector3d p_real;
Eigen::Vector3d pd_real;
std::atomic<bool> running{true};

void signal_handler(int signum) {
    (void)signum;
    running = false;
}






struct ExperimentParams {
    // Test configuration
    int number_trajectories = 1;
    int number_intrusions_per_trajectory = 1;
    
    // Qv values (weight for velocity error in optimization)
    std::vector<double> Qvs = {
        0.06, 0.07, 0.08, 0.09, 0.1,
        0.11, 0.125, 0.15, 0.2, 0.3
    };
    
    // PFL velocity values (velocity scaling factor)
    std::vector<double> vel_PFL = {0.1, 0.2, 0.3, 0.4, 0.5};
    
    // Timing
    double time_beginning = 0.0;
    double time_final = 10.0;
    double computational_frequency = 16.0;  // Hz
    double stopping_time = 0.3;  // seconds
    double pause_after_collision = 2.0;  // seconds
    
    double computational_period() const {
        return 1.0 / computational_frequency;
    }
};


double rms(const std::vector<double>& values) {
    if (values.empty()) return 0.0;
    
    double sum_squares = 0.0;
    for (double v : values) {
        sum_squares += v * v;
    }
    
    return std::sqrt(sum_squares / values.size());
}




// ─────────────────────────────────────────────────────────────────────────────
// SSM + PFL + Escape Trajectories strategy
// ─────────────────────────────────────────────────────────────────────────────
int SSM_PFL_escape(RobotModel& robot, 
        const std::array<Eigen::VectorXd, 2> q_r, 
        const std::array<Eigen::VectorXd, 2> qd_r, 
        const std::array<Eigen::VectorXd, 2> qdd_r,
        const Eigen::Vector3d p_h, 
        const Eigen::Vector3d pd_h, 
        const Eigen::Vector3d pdd_h) {
    
    // Initialize parameters and utilities
    ExperimentParams params;
    
    const double dt = params.computational_period();
    const double t_begin = params.time_beginning;
    const double t_final = params.time_final;
    
    // Create time vectors
    std::vector<double> time_points;
    for (double t = t_begin; t <= t_final + 1e-9; t += dt) {
        time_points.push_back(t);
    }
    
    std::vector<double> time_points_complete;
    for (double t = t_begin; t <= t_final + 25.0 + 1e-9; t += dt) {
        time_points_complete.push_back(t);
    }
    
    const int number_time_points = static_cast<int>(time_points.size());
    const int number_time_points_complete = static_cast<int>(time_points_complete.size());
    const int number_QpQv = static_cast<int>(params.Qvs.size());
    const int numberPFL = static_cast<int>(params.vel_PFL.size());

    std::cout << "HERE" << std::endl;

    Eigen::MatrixXd J = robot.ComputeJacobian("panda_link8", q_r[0]);
    // std::cout << "Jacobian (6x7):\n" << J << std::endl;

    std::array<Eigen::Vector3d, 2> p_r;
    p_r[0] = robot.GetJointPose("panda_link8", q_r[0]).translation().transpose();
    p_r[1] = robot.GetJointPose("panda_link8", q_r[1]).translation().transpose();
    std::array<Eigen::Vector3d, 2> pd_r;
    pd_r[0] = (J * qd_r[0]).tail<3>();
    pd_r[1] = (J * qd_r[1]).tail<3>();

    // Initialize simulation state
    if (do_once) {
        q_real = q_r[0];
        qd_real = qd_r[0];
        qdd_real = Eigen::VectorXd::Zero(7);
        p_real = p_r[0];
        pd_real = pd_r[0];
        do_once = false;
    }

    double velocity_PFL = 0.4;
    double Qv = 0.08;
    double HR_clearance = 0.1;
                
    std::vector<double> pd_real_module;
    std::vector<Eigen::VectorXd> qdd_real_history;
    
    bool collision = false;
    int collision_counter = 0;
    int arrival_step = number_time_points_complete;
    int failure_flag = 0;

    if (!collision) {
        // Compute safety distance delta
        double delta_safety = HR_clearance + pd_h.norm() * params.stopping_time;
        double velocity_term = -( -(delta_safety / params.stopping_time) + velocity_PFL ) * params.stopping_time;

        // std::cout << "=================================" << std::endl;
        // std::cout << q_real << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << qd_real << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << p_r[1] << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << pd_r[1] << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << p_h << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << pd_h << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << velocity_term << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << q_r[1] << std::endl;
        // std::cout << "---------------------------------" << std::endl;
        // std::cout << Qv << std::endl;
        // std::cout << "---------------------------------" << std::endl;

        SSMPFLResult res = SSMPFL(robot, dt, params.stopping_time, q_real, qd_real, p_r[1], pd_r[1], p_h, pd_h, velocity_term, q_r[1], Qv);
                            
        // Update state
        qdd_real = res.qdd_next;
        qd_real = res.qd_next;
        q_real = res.q_next;
        p_real = res.p_next;
        pd_real = res.pd_next;
        
        pd_real_module.push_back(pd_real.norm());
        qdd_real_history.push_back(qdd_real);
        
        // Check if optimization succeeded
        if (res.exitflag) {
            // Check collision with human (distance <= 0.1m)
            if ((p_real - p_h).norm() <= 0.1) {
                // R_STOP[w1][w2][w3][w4] += 1.0;
                collision = true;
                collision_counter = 0;
                std::cout << "===================== Collision detected" << std::endl;
                // std::cout << "    Qv=" << Qv << ", PFL=" << velocity_PFL << " -> Collision at step " << i << std::endl;
            }
            
            // // Check if robot reached goal position
            // if ((p_real - p_r).norm() <= 0.005) {
            //     // arrival_step = i;
            //     std::cout << "Trajectory completed" << std::endl;
            //     // std::cout << "    Qv=" << Qv << ", PFL=" << velocity_PFL << " -> Arrived at t=" << (i * dt) << "s" << std::endl;
            //     return 0;
            // }
        } else {
            // Optimization failed
            failure_flag = 1;
            std::cout << "Optimization failed" << std::endl;
            // std::cout << "    Qv=" << Qv << ", PFL=" << velocity_PFL << " -> Optimization failed" << std::endl;
            return 1;
        }
    } else {
        // Collision recovery phase
        collision_counter++;
        qdd_real.setZero();
        qd_real.setZero();
        // q_real.setZero();
        // p_real.setZero();
        pd_real.setZero();
        pd_real_module.push_back(0.0);
        
        if (collision_counter > params.pause_after_collision * params.computational_frequency) {
            collision = false;
            // reference_time = 0;
        }
    }

    
    return 0;
}



// ─────────────────────────────────────────────────────────────────────────────
// Main loop implementing chosen strategy
// ─────────────────────────────────────────────────────────────────────────────
int task_engine(
        std::vector<std::unique_ptr<DataTransmitter>>& transmitters, 
        RobotModel robot,
        int elapsed_ms, 
        const std::array<Eigen::VectorXd, 2> q_r, 
        const std::array<Eigen::VectorXd, 2> qd_r, 
        const std::array<Eigen::VectorXd, 2> qdd_r,
        Eigen::Vector3d& p_h, 
        Eigen::Vector3d& pd_h, 
        Eigen::Vector3d& pdd_h) {
    std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    std::optional<DistanceResult> dist = human_to_robot_distance(skeleton, robot, q_r[0]);

    // if (!dist) return 1;
    // else std::cout << "Minimum distance between robot and skeleton: " << dist->length << std::endl;

    double loop_duration = 0.001 * static_cast<double>(elapsed_ms);
    Eigen::VectorXd p_h_prev = p_h;
    Eigen::VectorXd pd_h_prev = pd_h;
    p_h = dist->c_h;
    pd_h = (p_h - p_h_prev)/loop_duration;
    pdd_h = (pd_h - pd_h_prev)/loop_duration;

    SSM_PFL_escape(robot, q_r, qd_r, qdd_r, p_h, pd_h, pdd_h);

    std::vector<nlohmann::json> payload;
    payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
    payload.push_back(q_real);
    payload.push_back(std::vector<int>{});
    transmitters[1]->send_data(payload);

    payload.clear();
    payload.push_back(dist->c_h);
    payload.push_back(dist->c_r);
    transmitters[2]->send_data(payload);
    
    return 0;
};


// ─────────────────────────────────────────────────────────────────────────────
// Load trajectory
// ─────────────────────────────────────────────────────────────────────────────
std::optional<Trajectory> load_trajectory(int n_traj, std::string c_dir) {
    std::string trajectory_path = c_dir + "src/trajectories/test" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    try {
        if (!f) throw 1;
    }
    catch (int err) {
        std::cerr << "Error: cannot open '" + trajectory_path + "'" << std::endl;
        return std::nullopt;
    }

    std::vector<std::array<double, 7>> traj_low = load_trajectory_CSV(trajectory_path + "q_ref.csv");
    std::vector<double> t_low = load_timestamps_CSV(trajectory_path + "t_ref.csv");
    Trajectory traj_high = interpolate_to_1kHz_full(traj_low, t_low);
    save_trajectory_CSV(trajectory_path + "q.csv",  traj_high.q);
    save_trajectory_CSV(trajectory_path + "qd.csv",  traj_high.qd);
    save_trajectory_CSV(trajectory_path + "qdd.csv",  traj_high.qdd);

    return traj_high;
}



// ─────────────────────────────────────────────────────────────────────────────
// Execute task
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, std::string c_dir="") {
    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(3);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));

    auto traj = load_trajectory(n_traj, c_dir);
    if (!traj) return 1;

    const std::string urdf_path = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf_path);

    // ── Definition of human collision point ──────────────────────────────────────
    Eigen::VectorXd q(Eigen::Map<Eigen::VectorXd>((*traj).q[0].data(), (*traj).q[0].size()));
    std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    std::optional<DistanceResult> dist = human_to_robot_distance(skeleton, robot, q);
    Eigen::Vector3d p_h = dist->c_h;
    Eigen::Vector3d pd_h = Eigen::VectorXd::Zero(3);
    Eigen::Vector3d pdd_h = Eigen::VectorXd::Zero(3);

    const int rate_hz = 16;
    const int period_ms = static_cast<int>(1.0 / rate_hz * 1000.0);
    auto next_time = std::chrono::steady_clock::now();
    auto loop_start = std::chrono::steady_clock::now();
    
    // ── Task engine ──────────────────────────────────────────────────────────────
    while (running) {
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        int elapsed_ms = static_cast<int>(std::round(std::chrono::duration<double>(elapsed).count() * 1000));
        if (elapsed_ms < traj->q.size()) {
            std::array<Eigen::VectorXd, 2> q_r;
            q_r[0] = Eigen::Map<Eigen::VectorXd>((*traj).q[elapsed_ms].data(), (*traj).q[elapsed_ms].size());
            q_r[1] = Eigen::Map<Eigen::VectorXd>((*traj).q[elapsed_ms + period_ms].data(), (*traj).q[elapsed_ms + period_ms].size());
            std::array<Eigen::VectorXd, 2> qd_r;
            qd_r[0] = Eigen::Map<Eigen::VectorXd>((*traj).qd[elapsed_ms].data(), (*traj).qd[elapsed_ms].size());
            qd_r[1] = Eigen::Map<Eigen::VectorXd>((*traj).qd[elapsed_ms + period_ms].data(), (*traj).qd[elapsed_ms + period_ms].size());
            std::array<Eigen::VectorXd, 2> qdd_r;
            qdd_r[0] = Eigen::Map<Eigen::VectorXd>((*traj).qdd[elapsed_ms].data(), (*traj).qdd[elapsed_ms].size());
            qdd_r[1] = Eigen::Map<Eigen::VectorXd>((*traj).qdd[elapsed_ms + period_ms].data(), (*traj).qdd[elapsed_ms + period_ms].size());
            
            task_engine(transmitters, robot, elapsed_ms, q_r, qd_r, qdd_r, p_h, pd_h, pdd_h);
        }
        else {
            loop_start = std::chrono::steady_clock::now();
        }
        
        next_time += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next_time);
    }

    for (auto &transmitter : transmitters) {
        transmitter->shutdown();
    }

    return 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    int n_traj = 0;
    std::string path;
    if (argc > 1) {
        try {
            std::string arg(argv[1]);
            const char* begin = arg.data();
            const char* end   = arg.data() + arg.size();
            auto [ptr, ec] = std::from_chars(begin, end, n_traj);
            if (ec != std::errc{} || ptr != end)
                throw 1;
        }
        catch (int err) {
            std::cerr << "Error: argument must be a valid integer." << std::endl;
            return 1;
        }
    }
    else {
        std::cerr << "Error: argument required." << std::endl;
        return 1;
    }
    if (argc > 2) {
        std::string c_dir(argv[2]);
        path = c_dir + "/";
    } 
    else {
        std::string c_dir(get_current_dir_name());
        path = c_dir + "/../";
    }

    std::signal(SIGINT, signal_handler);

    if (execute_task(n_traj, path)) return 1;
    else printf("Exiting cleanly...\n");
    
    return 0;
}
