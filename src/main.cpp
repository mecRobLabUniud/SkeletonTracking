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

#include "SSMPFL.hpp"

// ─────────────────────────────────────────────────────────────────────────────
bool do_once = true;
bool collision = false;
Eigen::VectorXd q_real;
Eigen::VectorXd qd_real;
Eigen::VectorXd qdd_real;
Eigen::Vector3d p_real;
Eigen::Vector3d pd_real;
std::atomic<bool> running{true};
int collision_counter = 0;

void signal_handler(int signum) {
    (void)signum;
    running = false;
}

struct ExperimentParams {
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
        const std::vector<Eigen::Vector3d> skeleton,
        const std::vector<Eigen::Vector3d> skeletond,
        const std::vector<Eigen::Vector3d> skeletondd) {
    
    // Initialize parameters and utilities
    ExperimentParams params;
    
    const double dt = params.computational_period();

    Eigen::MatrixXd J = robot.ComputeJacobian("panda_link8", q_r[0]);

    std::array<Eigen::Vector3d, 2> p_r;
    p_r[0] = robot.GetJointPose("panda_link8", q_r[0]).translation().transpose();
    p_r[1] = robot.GetJointPose("panda_link8", q_r[1]).translation().transpose();
    std::array<Eigen::Vector3d, 2> pd_r;
    pd_r[0] = (J * qd_r[0]).tail<3>();
    pd_r[1] = (J * qd_r[1]).tail<3>();

    double velocity_PFL = 0.1;
    double Qv = 0.008;
    double HR_clearance = 0.1;
    
    
    
    int arrival_step = number_time_points_complete;
    int failure_flag = 0;

    if (!collision) {
        for (int i=0; i<skeleton.size(); i++) {
            if (std::isnan(skeleton[i][0]) || std::isnan(skeleton[i][1]) || std::isnan(skeleton[i][2])) continue;

            // Compute safety distance delta
            double delta_safety = HR_clearance + skeletond[i].norm() * params.stopping_time;
            double velocity_term = -( -(delta_safety / params.stopping_time) + velocity_PFL ) * params.stopping_time;
    
            SSMPFLResult res = SSMPFL(robot, dt, params.stopping_time, q_real, qd_real, p_r[1], pd_r[1], skeleton[i], skeletond[i], velocity_term, q_r[1], Qv);
            
            // Check if optimization succeeded
            if (res.exitflag) {
                // Check collision with human (distance <= 0.1m)
                if ((p_real - skeleton[i]).norm() <= 0.1) {
                    collision = true;
                    collision_counter = 0;
                    std::cout << "=== Collision detected ===" << std::endl;
                    break;
                    // std::cout << "    Qv=" << Qv << ", PFL=" << velocity_PFL << " -> Collision at step " << i << std::endl;
                }
            } else {
                // Optimization failed
                failure_flag = 1;
                std::cout << "Optimization failed" << std::endl;
                // std::cout << "    Qv=" << Qv << ", PFL=" << velocity_PFL << " -> Optimization failed" << std::endl;
                return 1;
            }

            // Update state
            qdd_real = res.qdd_next;
            qd_real = res.qd_next;
            q_real = res.q_next;
            p_real = res.p_next;
            pd_real = res.pd_next;
        }
    } else {
        // Collision recovery phase
        collision_counter++;
        qdd_real.setZero();
        qd_real.setZero();
        // q_real.setZero();
        // p_real.setZero();
        pd_real.setZero();
        
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
        std::vector<Eigen::Vector3d>& skeleton,
        std::vector<Eigen::Vector3d>& skeletond,
        std::vector<Eigen::Vector3d>& skeletondd) {
    std::vector<Eigen::Vector3d> skeleton_prev = skeleton;
    std::vector<Eigen::Vector3d> skeletond_prev = skeletond;
    skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);

    std::optional<DistanceResult> dist;
    if (q_real.size() == 7) {
        dist = human_to_robot_distance(skeleton, robot, q_real);
    }    

    double loop_duration = 0.001 * static_cast<double>(elapsed_ms);

    for (int i=0; i<skeleton.size(); i++) {
        if (std::isnan(skeleton[i][0]) || std::isnan(skeleton[i][1]) || std::isnan(skeleton[i][2])) continue;
        
        skeletond[i] = (skeleton[i] - skeleton_prev[i])/loop_duration;
        skeletondd[i] = (skeletond[i] - skeletond_prev[i])/loop_duration;
    }

    SSM_PFL_escape(robot, q_r, qd_r, qdd_r, skeleton, skeletond, skeletondd);

    std::vector<nlohmann::json> payload;
    payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
    payload.push_back(std::vector<double>(q_real.data(), q_real.data() + q_real.size()));
    payload.push_back(std::vector<int>{});
    transmitters[1]->send_data(payload);

    payload.clear();
    payload.push_back(std::array<double, 3>{{dist->c_h[0], dist->c_h[1], dist->c_h[2]}});
    payload.push_back(std::array<double, 3>{{dist->c_r[0], dist->c_r[1], dist->c_r[2]}});
    transmitters[2]->send_data(payload);

    Eigen::Vector3d p_r;
    p_r = robot.GetJointPose("panda_link8", q_r[0]).translation().transpose();

    payload.clear();
    payload.push_back(std::vector<double>(p_real.data(), p_real.data() + p_real.size()));
    payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
    transmitters[3]->send_data(payload);
    
    return 0;
};


// ─────────────────────────────────────────────────────────────────────────────
// Load trajectory
// ─────────────────────────────────────────────────────────────────────────────
std::optional<Trajectory> load_trajectory(int n_traj, std::string c_dir, double t_start = 0.0, Eigen::VectorXd q_start = {}) {
    std::string trajectory_path = c_dir + "src/trajectories/test" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    if (!f) {
        std::cerr << "Error: cannot open '" << trajectory_path << "'" << std::endl;
        return std::nullopt;
    }

    std::vector<std::array<double, 7>> traj_low = load_trajectory_CSV(trajectory_path + "q_ref.csv");
    std::vector<double> t_low = load_timestamps_CSV(trajectory_path + "t_ref.csv");

    if (t_start != 0.0) {
        if (q_start.size() != 7) {
            std::cerr << "Error: q_start must have 7 elements when t_start != 0" << std::endl;
            return std::nullopt;
        }

        std::array<double, 7> q_start_arr;
        Eigen::VectorXd::Map(q_start_arr.data(), 7) = q_start;

        std::vector<std::array<double, 7>> traj_low_new;
        std::vector<double> t_low_new;
        traj_low_new.push_back(q_start_arr);
        t_low_new.push_back(0.0);

        for (int i = 0; i < traj_low.size(); i++) {
            if (t_start < t_low[i]) {
                traj_low_new.push_back(traj_low[i]);
                t_low_new.push_back(t_low[i] - t_start);
            }
        }

        traj_low = traj_low_new;
        t_low = t_low_new;
    }

    std::cout << "==================================== " << std::endl;
    std::cout << "\n";
    for (size_t i = 0; i < traj_low.size(); i++) {
        for (int j=0; j<7; j++) {
            std::cout << traj_low[i][j] << ", ";
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    Trajectory traj_high = interpolate_to_1kHz_full(traj_low, t_low);
    // save_trajectory_CSV(trajectory_path + "q.csv",  traj_high.q);
    // save_trajectory_CSV(trajectory_path + "qd.csv",  traj_high.qd);
    // save_trajectory_CSV(trajectory_path + "qdd.csv",  traj_high.qdd);

    return traj_high;
}


// ─────────────────────────────────────────────────────────────────────────────
// Execute task
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, std::string c_dir="") {
    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(4);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // auto traj = load_trajectory(n_traj, c_dir);
    // if (!traj) return 1;

    const std::string urdf_path = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf_path);

    // ── Definition of human skeleton points ──────────────────────────────────────
    std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    std::vector<Eigen::Vector3d> skeletond(skeleton.size(), Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> skeletondd(skeleton.size(), Eigen::Vector3d::Zero());

    const int rate_hz = 16;
    const int period_ms = static_cast<int>(1.0 / rate_hz * 1000.0);
    auto next_time = std::chrono::steady_clock::now();
    auto loop_start = std::chrono::steady_clock::now();
    auto delay_time_start = std::chrono::steady_clock::now();
    std::chrono::duration<double> delay_time{0.0};

    // ── Delay for loading web interface ──────────────────────────────────────────
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 4.0) {;} 
    
    next_time = std::chrono::steady_clock::now();
    loop_start = std::chrono::steady_clock::now();
    // delay_time_start = std::chrono::steady_clock::now();

    auto traj = load_trajectory(n_traj, c_dir);
    if (!traj) return 1;

    Eigen::VectorXd q_start = traj->q[0];
    double t_start = 0.0;
    int traj_size = traj->q.size();

    // ── Trajectory loop ──────────────────────────────────────────────────────────
    while (running) {
        start:
        std::cout << "t_start = " << t_start << std::endl;
        traj = load_trajectory(n_traj, c_dir, t_start, q_start);
        if (!traj) return 1;
        
        // Initialize simulation state
        q_real = traj->q[0];
        qd_real = traj->qd[0];
        qdd_real = traj->qdd[0];
        p_real = robot.GetJointPose("panda_link8", traj->q[0]).translation().transpose();
        pd_real = robot.GetJointPose("panda_link8", traj->qd[0]).translation().transpose();

        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        int elapsed_ms = static_cast<int>(std::round(std::chrono::duration<double>(elapsed).count() * 1000));

        std::cout << "==========================" << std::endl;
        std::cout << "elapsed_ms " << elapsed_ms << std::endl;
        std::cout << "traj_size " << traj_size << std::endl;
        std::cout << "==========================" << std::endl;

        while (elapsed_ms < traj_size) {
            elapsed = std::chrono::steady_clock::now() - loop_start;
            elapsed_ms = static_cast<int>(std::round(std::chrono::duration<double>(elapsed).count() * 1000));
            
            std::cout << "elapsed_ms " << elapsed_ms << std::endl;
            std::cout << "period_ms " << period_ms << std::endl;

            if (elapsed_ms + period_ms <= traj_size) {
                std::array<Eigen::VectorXd, 2> q_r;
                q_r[0] = traj->q[elapsed_ms - t_start*1000.0];
                q_r[1] = traj->q[elapsed_ms + period_ms - t_start*1000.0];
                std::array<Eigen::VectorXd, 2> qd_r;
                qd_r[0] = traj->qd[elapsed_ms - t_start*1000.0];
                qd_r[1] = traj->qd[elapsed_ms + period_ms - t_start*1000.0];
                std::array<Eigen::VectorXd, 2> qdd_r;
                qdd_r[0] = traj->qdd[elapsed_ms - t_start*1000.0];
                qdd_r[1] = traj->qdd[elapsed_ms + period_ms - t_start*1000.0];
                
                if (!collision) {
                    task_engine(transmitters, robot, elapsed_ms, q_r, qd_r, qdd_r, skeleton, skeletond, skeletondd);
                }
                else {
                    std::cout << "++++++++ collision +++++++++++" << std::endl;
                    auto collision_time = std::chrono::steady_clock::now();
                    while (collision) {         
                        task_engine(transmitters, robot, elapsed_ms, q_r, qd_r, qdd_r, skeleton, skeletond, skeletondd);
                    }
                    auto delay_time = std::chrono::steady_clock::now() - collision_time;
                    loop_start += delay_time;
                    next_time += delay_time;
                    q_start = q_r[0];
                    t_start = elapsed_ms/1000.0;


                    goto start;
                    // delay_time = std::chrono::steady_clock::now() - delay_time_start;
                }
            }

            next_time += std::chrono::milliseconds(period_ms);
            std::this_thread::sleep_until(next_time);
        }

        finish:
        std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
        std::cout << "+++++ Trajectory completed +++++" << std::endl;
        std::cout << "++++++++++++++++++++++++++++++++" << std::endl;

        next_time = std::chrono::steady_clock::now();
        loop_start = std::chrono::steady_clock::now();
        t_start = 0.0;
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