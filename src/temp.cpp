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

#include "SSMPFL.hpp"


// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt = 1.0 / rate_hz;
const double stopping_time = 0.25;
const double velocity_PFL = 0.4;
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
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 4) {;} 
        
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
    int r = 1;
    std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    Eigen::Vector3d ro;
    Eigen::Vector3d vo;
    std::vector<Eigen::Vector3d> skeleton_prev = skeleton;

    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();

    for (int i = 1; i <= n_steps; ++i) {
        const int k = i - 1;

        if (!collision) {
            const int rr = r;

            skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);

            // ── Core SSM, PFL and escape trajectory computation ─────────────
            Trajectory temp;
            temp.q.push_back(nominal_traj.q[rr]);
            temp.qd.push_back(nominal_traj.qd[rr]);
            temp.qdd.push_back(nominal_traj.qdd[rr]);
            temp.p.push_back(nominal_traj.p[rr]);
            temp.pd.push_back(nominal_traj.pd[rr]);

            bool exitflag = false;
            int joint = 0;

            constexpr int kNumKeypoints = 11;
            std::array<bool, kNumKeypoints> valid_marker{};
            std::array<bool, kNumKeypoints> qp_ok{};
            std::array<Eigen::VectorXd, kNumKeypoints> res_q, res_qd, res_qdd;
            std::array<Eigen::Vector3d, kNumKeypoints> res_p, res_pd;
            std::array<Eigen::Vector3d, kNumKeypoints> obs_p, obs_v;

            // ── Parallel SSM+PFL evaluation over the skeleton keypoints ─────
            #pragma omp parallel num_threads(std::min(kNumKeypoints, omp_get_max_threads()))
            {
                RobotModel robot_local = robot;
                #pragma omp for schedule(static)
                for (int j = 0; j <= 10; ++j) {
                    if (std::isnan(skeleton[j][0])) {
                        continue;
                    }
                    valid_marker[j] = true;
                    const Eigen::Vector3d p_obs = skeleton[j];
                    const Eigen::Vector3d v_obs = (skeleton[j] - skeleton_prev[j]) / dt;
                    obs_p[j] = p_obs;
                    obs_v[j] = v_obs;

                    const double delta = -(-(HR_clearance + v_obs.norm() * stopping_time) / stopping_time + velocity_PFL) * stopping_time;

                    SSMPFLResult res = SSMPFL(robot_local, dt, stopping_time, real_traj.q[k], real_traj.qd[k],
                                            nominal_traj.p[rr], nominal_traj.pd[rr], nominal_traj.q[rr],
                                            p_obs, v_obs, delta, Qpj, Qpt, Qv);

                    qp_ok[j] = res.exitflag;
                    res_q[j] = res.q_next;
                    res_qd[j] = res.qd_next;
                    res_qdd[j] = res.qdd_next;
                    res_p[j] = res.p_next;
                    res_pd[j] = res.pd_next;
                }
            }

            // ── Serial merge preserving the original best-result selection ──
            for (int j = 0; j <= 10; ++j) {
                if (!qp_ok[j]) continue;

                if ((res_qdd[j] - nominal_traj.qdd[rr]).norm() > (temp.qdd[0] - nominal_traj.qdd[rr]).norm()) {
                    temp.q[0] = res_q[j];
                    temp.qd[0] = res_qd[j];
                    temp.qdd[0] = res_qdd[j];
                    temp.p[0] = res_p[j];
                    temp.pd[0] = res_pd[j];

                    exitflag = true;
                    joint = j;

                    if (j != 1) {
                        std::cout << "Joint " << j << " -> " << (res_qdd[j] - nominal_traj.qdd[rr]).norm() 
                        << " > " << (temp.qdd[0] - nominal_traj.qdd[rr]).norm() << std::endl;
                    }
                }
            }

            

            // ── Obstacle state for publishing = last valid keypoint ─────────
            for (int j = 10; j >= 0; --j) {
                if (valid_marker[j]) {
                    ro = obs_p[j];
                    vo = obs_v[j];
                    break;
                }
            }
            
            real_traj.q.push_back(temp.q[0]);
            real_traj.qd.push_back(temp.qd[0]);
            real_traj.qdd.push_back(temp.qdd[0]);
            real_traj.p.push_back(temp.p[0]);
            real_traj.pd.push_back(temp.pd[0]);

            ++r;

            // ── Publish robot state, obstacle distance and trajectory ───────
            std::vector<nlohmann::json> payload;
            payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
            payload.push_back(std::vector<double>(real_traj.q[k].data(), real_traj.q[k].data() + real_traj.q[k].size()));
            payload.push_back(std::vector<int>{});
            transmitters[1]->send_data(payload);

            payload.clear();
            payload.push_back(std::array<double, 3>{{skeleton[joint][0], skeleton[joint][1], skeleton[joint][2]}});
            payload.push_back(std::array<double, 3>{{real_traj.p[k][0], real_traj.p[k][1], real_traj.p[k][2]}});
            transmitters[2]->send_data(payload);

            payload.clear();
            payload.push_back(std::vector<double>(real_traj.p[k].data(), real_traj.p[k].data() + real_traj.p[k].size()));
            payload.push_back(std::vector<double>(nominal_traj.p[k].data(), nominal_traj.p[k].data() + nominal_traj.p[k].size()));
            transmitters[3]->send_data(payload);

            skeleton_prev = skeleton;

            if (exitflag) {
                if (!CollisionFree(robot, real_traj.q[k], ro, HR_clearance)) {
                collision = true;
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
            const int prev = k > 0 ? k - 1 : k;
            real_traj.q.push_back(real_traj.q[prev]);
            real_traj.p.push_back(real_traj.p[prev]);
            real_traj.pd.push_back(Eigen::Vector3d::Zero());
            collision = false;
        }

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
