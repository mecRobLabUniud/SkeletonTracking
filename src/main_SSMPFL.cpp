#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <charconv>
#include <cmath>
#include <algorithm>
#include <memory>
#include <omp.h>
#include <Eigen/Dense>

#include "trajectory_utils.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include "robot_model.hpp"


#include "core.hpp"


// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt = 1.0 / rate_hz;
const double stopping_time = 0.25;
const double velocity_PFL = 0.4;
const double pause_after_collision = 0.5;
const double HR_clearance = 0.1;
const double Qpj = 70.0;
const double Qpt = 1.0;
const double Qv = 0.08;


// ─────────────────────────────────────────────────────────────────────────────
// Euclidean distance from point p to the segment a-b
// ─────────────────────────────────────────────────────────────────────────────
static double PointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                                    const Eigen::Vector3d& b) {
    const Eigen::Vector3d ab = b - a;
    const double len2 = ab.squaredNorm();
    double t = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

// ─────────────────────────────────────────────────────────────────────────────
// Return true when clearance between the obstacle point and the robot links is
// maintained along the given configuration
// ─────────────────────────────────────────────────────────────────────────────
bool CollisionFree(const RobotModel& robot, const Eigen::VectorXd& q,
                    const Eigen::Vector3d& p, double clearance) {
    static const char* kLinks[] = {"panda_link1", "panda_link2", "panda_link3",
                                    "panda_link4", "panda_link5", "panda_link6",
                                    "panda_link7", "panda_link8"};
    robot.ComputeFK(q);
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();
    for (const char* name : kLinks) {
        const Eigen::Vector3d cur = robot.GetJointPose(name).translation();
        if (PointSegmentDistance(p, prev, cur) < clearance) return false;
        prev = cur;
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Run the simulated tracking task for the given trajectory index and working
// directory
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, std::string c_dir="") {
    const std::string urdf = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf);

    // ── Data transmitters ───────────────────────────────────────────────────
    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(4);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // ── Delay for loading the web interface ─────────────────────────────────
    auto loop_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 5) {;} 
        
    double time_final = 5.0;    

    const int N = static_cast<int>(std::lround(time_final / dt)) + 1;

    Trajectory nominal_traj = load_p2p_trajectory(robot, c_dir, n_traj, N, dt);

    const int n_steps = static_cast<int>(nominal_traj.t.size());

    // ── PFL, SSM and escape trajectory simulation ───────────────────────────
    Trajectory real_traj;
    real_traj.q.push_back(nominal_traj.q[0]);
    real_traj.qd.push_back(nominal_traj.qd[0]);
    real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
    real_traj.p.push_back(nominal_traj.p[0]);
    real_traj.pd.push_back(nominal_traj.pd[0]);

    bool collision = false;
    int collision_counter = 0;
    int r = 1;
    std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    Eigen::Vector3d ro;
    Eigen::Vector3d vo;
    std::vector<Eigen::Vector3d> skeleton_prev = skeleton;

    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();

    for (int i = 0; i <= n_steps-1; ++i) {
    // int i = 0;
    // while (i <= n_steps-1) {
    //     i++;
        auto t0 = std::chrono::steady_clock::now();

        if (!collision) {
            skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);

            // ── Core SSM, PFL and escape trajectory computation ─────────────
            Trajectory temp;
            temp.q.push_back(nominal_traj.q[r]);
            temp.qd.push_back(nominal_traj.qd[r]);
            temp.qdd.push_back(nominal_traj.qdd[r]);
            temp.p.push_back(nominal_traj.p[r]);
            temp.pd.push_back(nominal_traj.pd[r]);

            bool exitflag = true;
            int joint = 0;

            constexpr int kNumKeypoints = 11;
            std::array<bool, kNumKeypoints> valid_marker{};
            std::array<bool, kNumKeypoints> qp_ok{};
            std::array<Eigen::Vector3d, kNumKeypoints> obs_p, obs_v;

            // ── Parallel SSM+PFL evaluation over the skeleton keypoints ─────
            // #pragma omp parallel num_threads(std::min(kNumKeypoints, omp_get_max_threads())) 
            // {
                // #pragma omp parallel for // shared(qp_ok, res_q, res_qd, res_qdd, res_p, res_pd, obs_p, obs_v)// schedule(static)
                for (int j = 0; j < kNumKeypoints; j++) {
                    RobotModel robot_local = robot;
                    if (std::isnan(skeleton[j][0])) {
                        continue;
                    }
                    valid_marker[j] = true;
                    const Eigen::Vector3d p_obs = skeleton[j];
                    const Eigen::Vector3d v_obs = (skeleton[j] - skeleton_prev[j]) / dt;
                    obs_p[j] = p_obs;
                    obs_v[j] = v_obs;

                    const double delta = -(-(HR_clearance + v_obs.norm() * stopping_time) / stopping_time + velocity_PFL) * stopping_time;

                    bool res = SSMPFL(robot, dt, stopping_time, nominal_traj.q[r], nominal_traj.qd[r], 
                                        nominal_traj.qdd[r], ro, vo, delta, Qpj, Qpt, Qv);

                    qp_ok[j] = res;
                }
            // }

            // std::cout << "qp_ok = " ; 
            // for (int j = 0; j < kNumKeypoints; j++) {
            //     if (std::isnan(skeleton[j][0])) {
            //         continue;
            //     }
            //     std::cout << qp_ok[j] << ", "; 
            // }
            // std::cout << std::endl; 

            // ── Serial merge preserving the original best-result selection ──
            for (int j = 8; j < kNumKeypoints; j++) {
                if (std::isnan(skeleton[j][0])) {
                    continue;
                }
                if (!qp_ok[j]) {
                    // temp.q[0] = nominal_traj.q[r-1];
                    // temp.qd[0] = Eigen::VectorXd::Zero(7);
                    // temp.qdd[0] = Eigen::VectorXd::Zero(7);
                    // temp.p[0] = nominal_traj.p[r-1];
                    // temp.pd[0] = Eigen::Vector3d::Zero();

                    exitflag = false;
                    joint = j;

                    break;
                };
            }

            // std::cout << "exitflag = " << exitflag << std::endl; 
            
            real_traj.q.push_back(temp.q[0]);
            real_traj.qd.push_back(temp.qd[0]);
            real_traj.qdd.push_back(temp.qdd[0]);
            real_traj.p.push_back(temp.p[0]);
            real_traj.pd.push_back(temp.pd[0]);

            // auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0);
            // std::cout << "TEMP SSMPFL elapsed time = " << elapsed.count() << "\n";

            r++;

            skeleton_prev = skeleton;

            if (exitflag) {
                for (int j = 0; j < kNumKeypoints; j++) {
                    if (!valid_marker[j]) {
                        continue;
                    }
                    if (!CollisionFree(robot, real_traj.q[i], obs_p[j], HR_clearance)) {
                        collision = true;
                    }
                }
            } 
            else {
                collision = true;
            }
        } 
        else {
            // ── Pause while in collision ────────────────────────────────────
            real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
            real_traj.qd.push_back(Eigen::VectorXd::Zero(7));
            const int prev = i > 0 ? i - 1 : i;
            real_traj.q.push_back(real_traj.q[prev]);
            real_traj.p.push_back(real_traj.p[prev]);
            real_traj.pd.push_back(Eigen::Vector3d::Zero());
            if (collision_counter++ > pause_after_collision * rate_hz) {
                collision = false;
                collision_counter = 0;
            }
            // collision = false;
        }        

        // ── Publish robot state, obstacle distance and trajectory ───────
        std::vector<nlohmann::json> payload;
        payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
        payload.push_back(std::vector<double>(real_traj.q[i].data(), real_traj.q[i].data() + real_traj.q[i].size()));
        payload.push_back(std::vector<int>{});
        transmitters[1]->send_data(payload);

        // payload.clear();
        // payload.push_back(std::array<double, 3>{{skeleton[joint][0], skeleton[joint][1], skeleton[joint][2]}});
        // payload.push_back(std::array<double, 3>{{real_traj.p[i][0], real_traj.p[i][1], real_traj.p[i][2]}});
        // transmitters[2]->send_data(payload);

        payload.clear();
        payload.push_back(std::vector<double>(real_traj.p[i].data(), real_traj.p[i].data() + real_traj.p[i].size()));
        payload.push_back(std::vector<double>(nominal_traj.p[r].data(), nominal_traj.p[r].data() + nominal_traj.p[r].size()));
        payload.push_back(collision);
        transmitters[3]->send_data(payload);

        next_time += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next_time);
    }

    return 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Entry point: parse the trajectory index and working directory, then run the
// task
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
        catch (int) {
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

    if (execute_task(n_traj, path)) return 1;
    else printf("Exiting cleanly...\n");
    
    return 0;
}
