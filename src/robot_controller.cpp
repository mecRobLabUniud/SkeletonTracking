#include <array>
#include <vector>
#include <chrono>
#include <charconv>
#include <cmath>
#include <thread>
#include <algorithm>
#include <filesystem>
#include <Eigen/Dense>

#include <franka/robot.h>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>

#include "utils.hpp"
#include "data_transmitter.hpp"
#include "trajectory_utils.hpp"
#include "project_root.hpp"
#include "shared_state.h"
#include "robot_model.hpp"
#include "examples_common.hpp"

// struct Sample { double t; std::array<double,7> q, dq; };

// ── Parameters ──────────────────────────────────────────────────────────────
const double rate_hz = 60.0;
const double dt      = 1.0 / rate_hz;

/*std::array<double,7> hermite(const Sample& a, const Sample& b, double t) {
  double h = b.t - a.t;
    double s = (t - a.t) / h;
    double h00 =  2*s*s*s - 3*s*s + 1;
    double h10 =      s*s*s - 2*s*s + s;
    double h01 = -2*s*s*s + 3*s*s;
    double h11 =      s*s*s -   s*s;
    std::array<double,7> q;
    for (int i = 0; i < 7; ++i)
        q[i] = h00*a.q[i] + h10*h*a.dq[i] + h01*b.q[i] + h11*h*b.dq[i];
    return q;
}


std::array<double,7> quintic(const WP& a, const WP& b, double t) {
    double T = b.t - a.t, s = (t - a.t) / T;
    double s2=s*s, s3=s2*s, s4=s3*s, s5=s4*s;
    double h0 = 1 - 10*s3 + 15*s4 - 6*s5;
    double h1 = s - 6*s3 + 8*s4 - 3*s5;
    double h2 = 0.5*s2 - 1.5*s3 + 1.5*s4 - 0.5*s5;
    double h3 = 10*s3 - 15*s4 + 6*s5;
    double h4 = -4*s3 + 7*s4 - 3*s5;
    double h5 = 0.5*s3 - s4 + 0.5*s5;
    std::array<double,7> q;
    for (int i = 0; i < 7; ++i)
        q[i] = h0*a.q[i] + h1*T*a.dq[i] + h2*T*T*a.ddq[i]
            + h3*b.q[i] + h4*T*b.dq[i] + h5*T*T*b.ddq[i];
    return q;
}*/

static Waypoint receiveRobotStates(DataTransmitter& tx) {
    const std::vector<nlohmann::json> data = tx.receive_data();
    // std::cout << "data[2]: " << data[2] << std::endl;
    // std::cout << "data[3]: " << data[3] << std::endl;

    if (data.empty()) return {};
    Waypoint wp;
    wp.q = json_to_robotstate(data[2]);
    wp.qd = json_to_robotstate(data[3]);
    wp.qdd = json_to_robotstate(data[4]);
    return wp;
}


void robotStatesThread(DataTransmitter& transmitter, SpcsRingBuffer<Waypoint, 2>& wp_queue) {
    Waypoint wp;
    while (!stop_flag.load(std::memory_order_acquire)) {
        auto t1 = std::chrono::steady_clock::now();
        wp = receiveRobotStates(transmitter);

        

        wp_queue.push(wp);
        auto t2 = std::chrono::steady_clock::now();
        // std::cout << "\rt: " << static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) << std::endl;
    }
}


int controller(int n_traj, const std::filesystem::path& root) {
    // ── Kinematic model ─────────────────────────────────────────────────────
    const std::filesystem::path urdf = root / "src" / "urdf" / "panda.urdf";
    if (!std::filesystem::exists(urdf)) {
        std::cerr << "URDF not found: " << urdf.string() << "\n";
        return 1;
    }
    RobotModel robot_model(urdf.string());

    // ── Nominal trajectory construction ──────────────────────────────────────
    const std::filesystem::path traj_dir =
        root / "src" / "trajectories" / ("test" + std::to_string(n_traj));

    Trajectory nominal_traj;
    Trajectory real_traj;
    try {
        nominal_traj = load_trajectory(robot_model, root.string() + "/", n_traj, dt);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load the nominal trajectory from " << traj_dir << ": "
                  << e.what() << "\n";
        return 1;
    }

    // ── Franka robot initialization ─────────────────────────────────────────
    franka::Robot robot("172.16.0.2");
    robot.setCollisionBehavior(
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,18,18,16,14,12}}, {{20,20,18,18,16,14,12}},
        {{20,20,20,25,25,25}}, {{20,20,20,25,25,25}},
        {{20,20,20,25,25,25}}, {{20,20,20,25,25,25}});

    std::array<double, 7> q_ref;
    std::copy(nominal_traj.wp[0].q.begin(), nominal_traj.wp[0].q.end(), q_ref.begin());

    // std::cout << "nominal q = " << nominal_traj.wp[0].q.transpose() << std::endl; 
    // std::cout << "array q = "; for (auto p : q_start) {std::cout << p << ", ";}; std::cout << std::endl; 



    MotionGenerator motion_generator(0.3, q_ref);
    std::cout << "WARNING: This example will move the robot! "
            << "Please make sure to have the user stop button at hand!" << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    // robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // ── Data transmitters ───────────────────────────────────────────────────
    DataTransmitter transmitter = DataTransmitter(DataTransmitter::Mode::Receiver, 12, "ROBOT");

    SpcsRingBuffer<Waypoint, 2> wp_queue;
    std::thread states_thread(robotStatesThread, std::ref(transmitter), std::ref(wp_queue));

    Waypoint previous = nominal_traj.wp[0];
    Waypoint next = nominal_traj.wp[0];
    double time = 0.0;
    std::vector<double> vnorm;
    // std::array<Waypoint, 2> buf;
    int i = 0;
    double t = 0.0;
    try {
        robot.control([&](const franka::RobotState& state,
                        franka::Duration period) -> franka::JointPositions {

            // std::cout << "====================================" << std::endl;
            // std::cout << "IN Waypoint q: " << wp_queue.in().q.transpose() << std::endl;
            // std::cout << "OUT Waypoint q: " << wp_queue.out().q.transpose() << std::endl;
            // std::cout << "PREVIOUS Waypoint q: " << previous.q.transpose() << std::endl;
            // std::cout << "NEXT Waypoint q: " << next.q.transpose() << std::endl;


            Waypoint in_wp = wp_queue.in();
            if (!wp_queue.empty() && in_wp.q.size() == next.q.size() && in_wp.q != next.q) {
                previous = next;
                wp_queue.pop(next);

                

                build_p2p_traj(previous.q, next.q, &vnorm, real_traj, dt,
                    t, robot_model, 0.001, previous.qd, previous.qdd, next.qd, next.qdd);

                std::cout << "PREVIOUS Waypoint q: " << previous.q.transpose() << std::endl;
                std::cout << "NEXT Waypoint q: " << next.q.transpose() << std::endl;
                // t += dt;

                
            }

            if (!real_traj.wp.empty() && i < static_cast<int>(real_traj.wp.size())) {
                std::cout << "Waypoint q: " << real_traj.wp[i++].q.transpose() << std::endl;
            }
            

            std::copy(real_traj.wp[i++].q.begin(), real_traj.wp[i++].q.end(), q_ref.begin());
            franka::JointPositions q{q_ref};
            return q;

            // // thread 60 Hz (non RT): scrive nel buffer inattivo, poi swap
            // int idle = 1 - active.load(std::memory_order_acquire);
            // buf[idle] = build_blended_segment(// ultimo q_d, dq_d, nuovi waypoint);
            // active.store(idle, std::memory_order_release);
// 
            // // callback 1 kHz: legge solo buf[active], nessun lock né allocazione
// 
            // // loop RT (callback di robot.control)
            // time += period.toSec();
            // while (time >= w1.t) {
            // WP next;
            // if (queue.pop(next)) { w0 = w1; w1 = next; }
            // return franka::JointPositions(quintic(w0, w1, std::min(time, w1.t)));
        });
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
    }

    stop_flag.store(true, std::memory_order_release);
    states_thread.join();

    return 0;
}


// ─────────────────────────────────────────────────────────────────────────────
// Entry point: parse the trajectory index and working directory, then run the
// task
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    // ── Command line: <trajectory index> [repository root] ──────────────────
    int n_traj = -1;
    std::string root_arg;
    if (argc > 1) {
        try {
            std::string arg(argv[1]);
            const char* begin = arg.data();
            const char* end   = arg.data() + arg.size();
            auto [ptr, ec] = std::from_chars(begin, end, n_traj);
            if (ec != std::errc{} || ptr != end || n_traj < 0)
                throw 1;
        }
        catch (int) {
            std::cerr << "Error: the trajectory index must be a valid non-negative integer."
                      << std::endl;
            return 1;
        }
    }
    else {
        std::cerr << "Error: the trajectory index is required "
                     "(usage: " << argv[0] << " <traj_index> [repo_dir])." << std::endl;
        return 1;
    }
    if (argc > 2) root_arg = argv[2];

    const std::filesystem::path root = findProjectRoot(root_arg);

    try {
        if (controller(n_traj, root)) return 1;
    }
    catch (const std::exception& e) {
        // Anything unexpected exits cleanly with a clear message, instead of
        // terminating the process with a core dump.
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    printf("Exiting cleanly...\n");

    return 0;
}