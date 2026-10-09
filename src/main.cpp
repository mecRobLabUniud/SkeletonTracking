// C++ port of "Thesis - Different Methods" (PFL & SSM & escape branch only).
// Uses your RobotModel (Pinocchio) and SSMPFL() as given.
//
// Build (adapt to your CMake): link against Eigen3 + pinocchio.
//
// Differences vs MATLAB worth knowing:
//   * Jacobian ordering: Pinocchio is [linear; angular], MATLAB's
//     geometricJacobian is [angular; linear]. So "vf(4:6,:)" -> head<3>() here.
//   * COLLcheck_franka was not provided -> CollisionFree() below is a
//     placeholder (link-origin segments vs. a point with clearance). Replace
//     with your real check if you have one.
//   * Qpj / Qpt are new arguments of your C++ SSMPFL that the MATLAB call did
//     not have -> set them to whatever your MATLAB SSMPFL_franka used.

#include <chrono>
#include <cstdio>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <charconv>
#include <cmath>
#include <algorithm>
#include <memory>
#include <Eigen/Dense>

#include "trajectory_utils.hpp"
#include "data_transmitter.hpp"
#include "utils.hpp"
#include "robot_model.hpp"

#include "SSMPFL.hpp"




// ───────────────────────────────────────────────────────────────────────────
const double rate_hz = 20.0;
const double dt = 1.0 / rate_hz;
const double stopping_time = 0.25;
const double velocity_PFL = 0.4;
const double HR_clearance = 0.1;
const double Qpj = 70.0;
const double Qpt = 1.0;
const double Qv = 0.08;


// ----------------------------------------------------------------------------
// Placeholder for COLLcheck_franka. Returns TRUE when there is NO collision
// (matches the MATLAB usage: `if not(COLLcheck_franka(...))` -> collision).
// ----------------------------------------------------------------------------
static double PointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                                    const Eigen::Vector3d& b) {
    const Eigen::Vector3d ab = b - a;
    const double len2 = ab.squaredNorm();
    double t = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return (p - (a + t * ab)).norm();
}

bool CollisionFree(const RobotModel& robot, const Eigen::VectorXd& q,
                    const Eigen::Vector3d& p, double clearance) {
    static const char* kLinks[] = {"panda_link1", "panda_link2", "panda_link3",
                                    "panda_link4", "panda_link5", "panda_link6",
                                    "panda_link7", "panda_link8"};
    robot.ComputeFK(q);
    Eigen::Vector3d prev = Eigen::Vector3d::Zero();  // base origin
    for (const char* name : kLinks) {
        const Eigen::Vector3d cur = robot.GetJointPose(name).translation();
        if (PointSegmentDistance(p, prev, cur) < clearance) return false;
        prev = cur;
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Execute task
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, std::string c_dir="") {
    const std::string urdf = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf);

    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(4);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // ── Delay for loading web interface ──────────────────────────────────────────
    auto loop_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 4) {;} 
        
    double time_final = 5.0;    

    const int N = static_cast<int>(std::lround(time_final / dt)) + 1;

    Trajectory nominal_traj = load_p2p_trajectory(robot, c_dir, n_traj, N, dt);

    const int n_steps = static_cast<int>(nominal_traj.t.size());

    // ── PFL & SSM & escape traj simulation ───────────────────────────────────────
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
    // Eigen::Vector3d ro_prev;
    // Eigen::Vector3d vo_prev;

    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();

    for (int i = 1; i <= n_steps; ++i) {
        const int k = i - 1;  // index of the latest state (MATLAB column i)

        if (!collision) {
            const int rr = r;

            skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);





            

            // ── Core SSM + PFL + escape traj computation ───────────────────────────────
            Trajectory temp;
            temp.q.push_back(nominal_traj.q[rr]);
            temp.qd.push_back(nominal_traj.qd[rr]);
            temp.qdd.push_back(nominal_traj.qdd[rr]);
            temp.p.push_back(nominal_traj.p[rr]);
            temp.pd.push_back(nominal_traj.pd[rr]);

            bool exitflag = false;
            int joint = 0;

            for (int j=0; j<=10; j++) {
                if (std::isnan(skeleton[j][0])) {
                    continue;
                    // ro = ro_prev;
                    // vo = vo_prev;
                } else {
                    ro = skeleton[j];
                    vo = (skeleton[j] - skeleton_prev[j])/dt;
                }

                const double delta = -(-(HR_clearance + vo.norm() * stopping_time) / stopping_time + velocity_PFL) * stopping_time;

                SSMPFLResult res = SSMPFL(robot, dt, stopping_time, real_traj.q[k], real_traj.qd[k],
                                        nominal_traj.p[rr], nominal_traj.pd[rr], nominal_traj.q[rr],
                                        ro, vo, delta, Qpj, Qpt, Qv);

                
                // ── Check error between real and nominal qdd ─────────────────────────────────
                if (res.exitflag) {
                    if ((res.qdd_next - nominal_traj.qdd[rr]).norm() > (temp.qdd[0] - nominal_traj.qdd[rr]).norm()) { 
                        temp.q[0] = res.q_next;
                        temp.qd[0] = res.qd_next;
                        temp.qdd[0] = res.qdd_next;
                        temp.p[0] = res.p_next;
                        temp.pd[0] = res.pd_next;

                        exitflag = true;
                        joint = j;
                    }
                }
            }
            
            real_traj.q.push_back(temp.q[0]);
            real_traj.qd.push_back(temp.qd[0]);
            real_traj.qdd.push_back(temp.qdd[0]);
            real_traj.p.push_back(temp.p[0]);
            real_traj.pd.push_back(temp.pd[0]);

            std::cout << "Joint: " << joint << std::endl;






            ++r;

            std::vector<nlohmann::json> payload;
            payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
            payload.push_back(std::vector<double>(real_traj.q[k].data(), real_traj.q[k].data() + real_traj.q[k].size()));
            payload.push_back(std::vector<int>{});
            transmitters[1]->send_data(payload);

            payload.clear();
            payload.push_back(std::array<double, 3>{{ro[0], ro[1], ro[2]}});
            payload.push_back(std::array<double, 3>{{real_traj.p[k][0], real_traj.p[k][1], real_traj.p[k][2]}});
            transmitters[2]->send_data(payload);

            Eigen::Vector3d p_r;
            p_r = robot.GetJointPose("panda_link8", nominal_traj.q[rr]).translation().transpose();

            payload.clear();
            payload.push_back(std::vector<double>(real_traj.p[k].data(), real_traj.p[k].data() + real_traj.p[k].size()));
            payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
            transmitters[3]->send_data(payload);


            // ro_prev = ro;
            // vo_prev = vo;
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
            real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
            real_traj.qd.push_back(Eigen::VectorXd::Zero(7));
            // NOTE: faithful to MATLAB `real_traj.q(:,end-1)` / `real_traj.p(:,end-1)`, which
            // repeats the sample *before* the latest one. Use [k] instead if you
            // actually intend the robot to hold its current pose during the pause.
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