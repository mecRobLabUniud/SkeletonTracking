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
#include "quintic_trajectory.hpp"

#include "SSMPFL.hpp"

// ── Initializations ──────────────────────────────────────────────────────────
bool do_once = true;
bool collision = false;
Eigen::VectorXd q_real;
Eigen::VectorXd qd_real;
Eigen::VectorXd qdd_real;
Eigen::Vector3d p_real;
Eigen::Vector3d pd_real;
std::atomic<bool> running{true};
int collision_counter = 0;

// ── Parameters ───────────────────────────────────────────────────────────────
const int rate_hz = 60;
double velocity_PFL = 0.4;
double HR_clearance = 0.1;
double stopping_time = 0.25;
double pause_after_collision = 3.75;


double Qpj = 70.0;
double Qpt = 1.0;
double Qv = 0.08;


void signal_handler(int signum) {
    (void)signum;
    running = false;
}

double rms(const std::vector<double>& values) {
    if (values.empty()) return 0.0;

    double sum_squares = 0.0;
    for (double v : values) {
        sum_squares += v * v;
    }

    return std::sqrt(sum_squares / values.size());
}






// ----------------------------------------------------------------------------
// Quintic polynomial trajectory between two points (zero vel/acc at the ends),
// equivalent to quinticpolytraj(...) with 2 waypoints.
// ----------------------------------------------------------------------------
struct Traj {
  Eigen::MatrixXd p, v, a;  // dim x n
};

Traj QuinticPolyTraj(const Eigen::VectorXd& x0, const Eigen::VectorXd& x1, double T,
                     double dt, int n) {
  const int d = static_cast<int>(x0.size());
  Traj tr{Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n), Eigen::MatrixXd(d, n)};
  const Eigen::VectorXd dx = x1 - x0;
  for (int k = 0; k < n; ++k) {
    const double t = std::min(k * dt, T);
    const double s = t / T;
    const double s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    const double h = 10 * s3 - 15 * s4 + 6 * s5;
    const double dh = (30 * s2 - 60 * s3 + 30 * s4) / T;
    const double ddh = (60 * s - 180 * s2 + 120 * s3) / (T * T);
    tr.p.col(k) = x0 + dx * h;
    tr.v.col(k) = dx * dh;
    tr.a.col(k) = dx * ddh;
  }
  return tr;
}







// ─────────────────────────────────────────────────────────────────────────────
// SSM + PFL + Escape Trajectories strategy
// ─────────────────────────────────────────────────────────────────────────────
int SSM_PFL_escape(RobotModel& robot, 
        const std::array<Eigen::VectorXd, 2> q_r, 
        const std::array<Eigen::VectorXd, 2> qd_r,
        const std::vector<Eigen::Vector3d> skeleton,
        const std::vector<Eigen::Vector3d> skeletond) {
    
    // Initialize parameters and utilities    
    const double dt = 1.0 / static_cast<double>(rate_hz);

    Eigen::MatrixXd J = robot.ComputeJacobian("panda_link8", q_r[0]);

    std::array<Eigen::Vector3d, 2> p_r;
    p_r[0] = robot.GetJointPose("panda_link8", q_r[0]).translation().transpose();
    p_r[1] = robot.GetJointPose("panda_link8", q_r[1]).translation().transpose();
    std::array<Eigen::Vector3d, 2> pd_r;
    pd_r[0] = (J * qd_r[0]).tail<3>();
    pd_r[1] = (J * qd_r[1]).tail<3>();      
    
    int failure_flag = 0;

    if (!collision) {
        for (int i=0; i<skeleton.size(); i++) {
            if (std::isnan(skeleton[i][0]) || std::isnan(skeleton[i][1]) || std::isnan(skeleton[i][2])) continue;


            Eigen::Vector3d ro = skeleton[i];
            Eigen::Vector3d vo = skeletond[i];

            // Eigen::Vector3d ro = Eigen::Vector3d{0.5, 0.5, 0.3};
            // Eigen::Vector3d vo = Eigen::Vector3d{0.0, 0.0, 0.0};



            // Compute safety distance delta
            double delta_safety = HR_clearance + skeletond[i].norm()*stopping_time;
            double velocity_term = -( -(delta_safety/stopping_time) + velocity_PFL )*stopping_time;


            // std::cout << "delta_safety = " << delta_safety << std::endl;


            // std::cout << "dt = " << dt << std::endl; 
            // std::cout << "stopping_time = " << stopping_time << std::endl; 
            // std::cout << "q_real = " << q_real << std::endl; 
            // std::cout << "qd_real = " << qd_real << std::endl; 
            // std::cout << "p_r[1] = " << p_r[1] << std::endl; 
            // std::cout << "pd_r[1] = " << pd_r[1] << std::endl; 
            // std::cout << "q_r[1] = " << q_r[1] << std::endl; 
            // std::cout << "skeleton[i] = " << skeleton[i] << std::endl; 
            // std::cout << "skeletond[i] = " << skeletond[i] << std::endl; 
            // std::cout << "velocity_term = " << velocity_term << std::endl; 
            // std::cout << "Qpj = " << Qpj << std::endl; 
            // std::cout << "Qpt = " << Qpt << std::endl; 
            // std::cout << "Qv = " << Qv << std::endl; 

            // std::cout << "q_r+1 = " << q_r[1] << std::endl; 
            // std::cout << "qd_r = " << qd_r[0] << std::endl; 
            // std::cout << "qd_r+1 = " << qd_r[1] << std::endl; 
            // std::cout << "qdd_r = " << qdd_r[0] << std::endl; 
            // std::cout << "qdd_r+1 = " << qdd_r[1] << std::endl; 



            
            
            SSMPFLResult res = SSMPFL(robot, dt, stopping_time, q_real, qd_real, p_r[1], pd_r[1], q_r[1], ro, vo, velocity_term, Qpj, Qpt, Qv);
            
            // Check if optimization succeeded
            if (res.exitflag) {

                std::optional<DistanceResult> dist;
                if (q_real.size() == 7) {
                    dist = human_to_robot_distance(skeleton, robot, q_real);
                    // std::cout << "dist = " << dist->length << std::endl;
                } 

                // Check collision with human (distance <= 0.1m)
                // if ((p_real - skeleton[i]).norm() <= 0.1) {
                // if (dist->length <= 0.1) {
                //     collision = true;
                //     collision_counter = 0;
                //     std::cout << "=== Collision detected ===" << std::endl;
                //     break;
                // }
            } else {
                // Optimization failed
                failure_flag = 1;
                std::cout << "Optimization failed" << std::endl;
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
        pd_real.setZero();
        
        if (collision_counter > pause_after_collision * static_cast<double>(rate_hz)) {
            collision = false;
        }
    }

    return 0;
}

/*

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

    double loop_duration = 1.0/rate_hz;

    for (int i=0; i<skeleton.size(); i++) {
        if (std::isnan(skeleton[i][0]) || std::isnan(skeleton[i][1]) || std::isnan(skeleton[i][2])) continue;
        
        skeletond[i] = (skeleton[i] - skeleton_prev[i])*rate_hz;
        skeletondd[i] = (skeletond[i] - skeletond_prev[i])*rate_hz;
    }

    SSM_PFL_escape(robot, q_r, qd_r, skeleton, skeletond);

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
    std::string trajectory_path = c_dir + "src/trajectories/traj" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    if (!f) {
        std::cerr << "Error: cannot open '" << trajectory_path << "'" << std::endl;
        return std::nullopt;
    }

    std::vector<std::array<double, 7>> traj_low = load_trajectory_CSV(trajectory_path + "q_ref.csv");
    std::vector<double> t_low = load_timestamps_CSV(trajectory_path + "t_ref.csv");

    traj_low[0] = {{0.2082, 0.6204, -0.0830, -1.5158, 0.0570, 2.1339, 0.1102}};
    traj_low[1] = {{1.6896, 0.5525, -0.3451, -1.3974, 0.1899, 1.9174, 1.3274}};



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

    for (size_t i = 0; i < traj_low.size(); i++) {
        for (int j=0; j<7; j++) {
        }
    }

    // Trajectory traj_high = interpolate_to_1kHz_full(traj_low, t_low);
    Trajectory traj_high = interpolateQuintic(traj_low, t_low);

    // save_trajectory_CSV(trajectory_path + "q.csv",  traj_high.q);
    // save_trajectory_CSV(trajectory_path + "qd.csv",  traj_high.qd);
    // save_trajectory_CSV(trajectory_path + "qdd.csv",  traj_high.qdd);

    return traj_high;
}*/


// ─────────────────────────────────────────────────────────────────────────────
// Execute task
// ─────────────────────────────────────────────────────────────────────────────
int execute_task (int n_traj, std::string c_dir="") {
    // std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    // transmitters.reserve(4);
    // transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Receiver, 10, "MERGED"));
    // transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    // transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    // transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // auto traj = load_trajectory(n_traj, c_dir);
    // if (!traj) return 1;

    const std::string urdf_path = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf_path);

    // // ── Definition of human skeleton points ──────────────────────────────────────
    // std::vector<Eigen::Vector3d> skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);
    // std::vector<Eigen::Vector3d> skeletond(skeleton.size(), Eigen::Vector3d::Zero());
    // std::vector<Eigen::Vector3d> skeletondd(skeleton.size(), Eigen::Vector3d::Zero());

    const int period_ms = static_cast<int>(1000.0/rate_hz);
    auto next_time = std::chrono::steady_clock::now();
    auto loop_start = std::chrono::steady_clock::now();
    auto delay_time_start = std::chrono::steady_clock::now();
    std::chrono::duration<double> delay_time{0.0};

    // ── Delay for loading web interface ──────────────────────────────────────────
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 4.0) {;} 
    
    next_time = std::chrono::steady_clock::now();
    loop_start = std::chrono::steady_clock::now();
    // delay_time_start = std::chrono::steady_clock::now();

    /*auto traj = load_trajectory(n_traj, c_dir);
    if (!traj) return 1;

    // Eigen::VectorXd q_start = traj->q[0];
    // double t_start = 0.0;
    int traj_size = 2400; // traj->q.size();
    int cnt = 0;

    // Initialize simulation state
    q_real = traj->q[0];
    qd_real = traj->qd[0];
    qdd_real = traj->qdd[0];
    p_real = robot.GetJointPose("panda_link8", traj->q[0]).translation().transpose();
    pd_real = robot.GetJointPose("panda_link8", traj->qd[0]).translation().transpose();
*/




    const std::string ee = "panda_link8";

    std::vector<std::unique_ptr<DataTransmitter>> transmitters;
    transmitters.reserve(2);
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 12, "ROBOT"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 13, "DISTANCE"));
    transmitters.push_back(std::make_unique<DataTransmitter>(DataTransmitter::Mode::Sender, 14, "TRAJDATA"));

    // ---------------------------------------------------------------- Parameters
    const double time_final = 5.0;
    const double time_experiment = 30.0;
    const double freq = 150.0;
    const double dt = 1.0 / freq;
    const double stopping_time = 0.25;
    const double pause_after_collision = 3.75;
    const double velocity_PFL = 0.4;
    const double Qv_PFL = 0.08;
    const double HR_clearance = 0.1;
    const int n_steps = static_cast<int>(freq*time_experiment);  // MATLAB: for i=1:380

    // TODO: set to the values your MATLAB SSMPFL_franka used.
    const double Qpj = 70.0;
    const double Qpt = 1.0;

    const int N = static_cast<int>(std::lround(time_final / dt)) + 1;  // 1001
    const int Nc = static_cast<int>(std::lround(time_experiment / dt)) + 1;
    const int M = (N - 1) * 3 + 1;  // padded trajectory length

    const Eigen::Vector3d robot_start(0.7, 0.1, 0.3);
    const Eigen::Vector3d robot_end(0.1, 0.7, 0.4);

    Eigen::VectorXd q_base(7);
    q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;


    // --------------------------------------------------- Robot trajectory (IK)
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(1, 1) = -1.0;
    R(2, 2) = -1.0;
    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T2 = Eigen::Isometry3d::Identity();
    T1.linear() = R;  T1.translation() = robot_start;
    T2.linear() = R;  T2.translation() = robot_end;

    Eigen::VectorXd q1, q2;
    if (!robot.ComputeIK(ee, T1, q_base, &q1)) throw std::runtime_error("IK failed for start pose");
    if (!robot.ComputeIK(ee, T2, q_base, &q2)) throw std::runtime_error("IK failed for end pose");

    // Forward (q1->q2) and backward (q2->q1) joint trajectories, padded by holding
    // the last position (velocities/accelerations stay zero, as in MATLAB).
    auto build_joint_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& q,
                                Eigen::MatrixXd& qd, Eigen::MatrixXd& qdd) {
        Traj tr = QuinticPolyTraj(a, b, time_final, dt, N);
        q = Eigen::MatrixXd::Zero(7, M);
        qd = Eigen::MatrixXd::Zero(7, M);
        qdd = Eigen::MatrixXd::Zero(7, M);
        q.leftCols(N) = tr.p;
        qd.leftCols(N) = tr.v;
        qdd.leftCols(N) = tr.a;
        for (int k = N; k < M; ++k) q.col(k) = q.col(N - 1);
    };
    Eigen::MatrixXd qf, qdf, qddf, qs, qds, qdds;
    build_joint_traj(q1, q2, qf, qdf, qddf);
    build_joint_traj(q2, q1, qs, qds, qdds);

    // Cartesian analysis: position and 6D velocity (Pinocchio: [linear; angular]).
    auto build_cart = [&](const Eigen::MatrixXd& q, const Eigen::MatrixXd& qd, Eigen::MatrixXd& p,
                            Eigen::MatrixXd& v, std::vector<double>* vnorm) {
        p = Eigen::MatrixXd::Zero(3, M);
        v = Eigen::MatrixXd::Zero(6, M);  // stays zero past N, as in MATLAB
        for (int k = 0; k < N; ++k) {
        v.col(k) = robot.ComputeJacobian(ee, q.col(k)) * qd.col(k);
        p.col(k) = robot.GetJointPose(ee, q.col(k)).translation();
        if (vnorm) vnorm->push_back(v.col(k).norm());
        }
        for (int k = N; k < M; ++k) p.col(k) = p.col(N - 1);
    };
    Eigen::MatrixXd pf, vf, ps, vs;
    std::vector<double> v_norm;
    build_cart(qf, qdf, pf, vf, &v_norm);
    build_cart(qs, qds, ps, vs, nullptr);

    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    // ------------------------------------------------------ Human trajectory
    const double t_move = 1.75, t_pause = 1.5;
    const int Nm = static_cast<int>(std::lround(t_move * freq)) + 1;  // 251
    const int Nh = static_cast<int>(std::lround((2 * t_move + t_pause) * freq)) + 1;  // 1001

    const Eigen::Vector3d hA(0.8, 0.8, 0.3), hB(0.4, 0.4, 0.3);
    Traj hf = QuinticPolyTraj(hA, hB, t_move, dt, Nm);  // A -> B
    Traj hs = QuinticPolyTraj(hB, hA, t_move, dt, Nm);  // B -> A

    Eigen::MatrixXd p_unit(3, Nh), v_unit = Eigen::MatrixXd::Zero(3, Nh);
    for (int k = 0; k < Nh; ++k) p_unit.col(k) = hA;
    const int m = Nm - 1;  // 250: last index of first move / first of second
    p_unit.block(0, 0, 3, Nm) = hf.p;
    v_unit.block(0, 0, 3, Nm) = hf.v;
    p_unit.block(0, m, 3, Nm) = hs.p;
    v_unit.block(0, m, 3, Nm) = hs.v;

    Eigen::MatrixXd p_int = Eigen::MatrixXd::Zero(3, Nc), v_int = Eigen::MatrixXd::Zero(3, Nc);
    const int reps = static_cast<int>(std::lround(time_experiment / time_final));
    for (int j = 0; j < reps; ++j) {
        const int start = j * static_cast<int>(freq * time_final);
        p_int.block(0, start, 3, Nh) = p_unit;
        v_int.block(0, start, 3, Nh) = v_unit;
    }

    // -------------------------------------- PFL & SSM & Escape simulation
    auto t0 = std::chrono::steady_clock::now();

    // std::vector<Eigen::VectorXd> qdd_real{Eigen::VectorXd::Zero(7)};
    // std::vector<Eigen::VectorXd> qd_real{qdf.col(0)};
    // std::vector<Eigen::VectorXd> q_real{qf.col(0)};
    // std::vector<Eigen::Vector3d> p_real{pf.col(0)};
    // std::vector<Eigen::Vector3d> v_real{vf.col(0).head<3>()};
    std::vector<int> flag;

    qdd_real = Eigen::VectorXd::Zero(7);
    qd_real = qdf.col(0);
    q_real = qf.col(0);
    p_real = pf.col(0);
    pd_real = vf.col(0).head<3>();


    int collision_counter = 0;
    bool collision = false, forward = true;
    int cnt = 0;


    loop_start = std::chrono::steady_clock::now();
    // ── Delay for loading web interface ──────────────────────────────────────────
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 4.0) {;} 
    

    // ── Trajectory loop ──────────────────────────────────────────────────────────
    while (running) {
        // std::cout << "t_start = " << t_start << std::endl;
        // traj = load_trajectory(n_traj, c_dir, t_start, q_start);
        // if (!traj) return 1;
        
        
        auto elapsed = std::chrono::steady_clock::now() - loop_start;
        int elapsed_ms = static_cast<int>(std::round(std::chrono::duration<double>(elapsed).count() * 1000));

        // while (cnt < traj_size) {
        while (cnt++ < n_steps) {
            auto time1 = std::chrono::steady_clock::now();
            elapsed = std::chrono::steady_clock::now() - loop_start;
            elapsed_ms = static_cast<int>(std::round(std::chrono::duration<double>(elapsed).count() * 1000));

            // if (cnt + period_ms < traj_size) {
            if (true) {
                // std::cout << std::endl << std::endl << "++++++++ cnt = " << cnt << std::endl << std::endl << std::endl;
                // std::array<Eigen::VectorXd, 2> q_r;
                // q_r[0] = traj->q[cnt];
                // q_r[1] = traj->q[cnt + period_ms];
                // std::array<Eigen::VectorXd, 2> qd_r;
                // qd_r[0] = traj->qd[cnt];
                // qd_r[1] = traj->qd[cnt + period_ms];
                // std::array<Eigen::VectorXd, 2> qdd_r;
                // qdd_r[0] = traj->qdd[cnt];
                // qdd_r[1] = traj->qdd[cnt + period_ms];
                
                if (!collision) {
                    // task_engine(transmitters, robot, cnt, q_r, qd_r, qdd_r, skeleton, skeletond, skeletondd);
                    
                    const Eigen::MatrixXd& Pr = forward ? pf : ps;
                    const Eigen::MatrixXd& Qr = forward ? qf : qs;
                    const Eigen::MatrixXd& Qdr = forward ? qdf : qds;

                    // Eigen::Vector3d ro = p_int.col(cnt);
                    // Eigen::Vector3d vo = v_int.col(cnt);


                    // std::array<Eigen::VectorXd, 2> p_r;
                    // p_r[0] = Pr.col(cnt);
                    // p_r[1] = Pr.col(cnt+1);
                    std::array<Eigen::VectorXd, 2> q_r;
                    q_r[0] = Qr.col(cnt);
                    q_r[1] = Qr.col(cnt+1);
                    std::array<Eigen::VectorXd, 2> qd_r;
                    qd_r[0] = Qdr.col(cnt);
                    qd_r[1] = Qdr.col(cnt+1);
                    std::vector<Eigen::Vector3d> skeleton;
                    skeleton.push_back(p_int.col(cnt));
                    std::vector<Eigen::Vector3d> skeletond;
                    skeletond.push_back(v_int.col(cnt));



                    

                    SSM_PFL_escape(robot, q_r, qd_r, skeleton, skeletond);

                    





                    Eigen::Vector3d p_r;
                    p_r = robot.GetJointPose("panda_link8", q_r[0]).translation().transpose();

                    std::vector<nlohmann::json> payload;
                    payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
                    payload.push_back(std::vector<double>(q_real.data(), q_real.data() + q_real.size()));
                    payload.push_back(std::vector<int>{});
                    transmitters[0]->send_data(payload);

                    payload.clear();
                    payload.push_back(std::array<double, 3>{{skeleton[0][0], skeleton[0][1], skeleton[0][2]}});
                    payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
                    transmitters[1]->send_data(payload);

                    
                    payload.clear();
                    payload.push_back(std::vector<double>(p_real.data(), p_real.data() + p_real.size()));
                    payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
                    transmitters[2]->send_data(payload);

                
                    if (forward) {
                        if ((p_real - pf.col(N - 1)).norm() <= 0.01) {
                            cnt = 0;
                            forward = false;
                        }
                        } else {
                        if ((p_real - ps.col(N - 1)).norm() <= 0.01) {
                            cnt = 0;
                            forward = true;
                        }
                    }





                }
                else {
                    // std::cout << "++++++++ collision +++++++++++" << std::endl;
                    // auto collision_time = std::chrono::steady_clock::now();
                    // while (collision) {        
                    //     task_engine(transmitters, robot, cnt, q_r, qd_r, qdd_r, skeleton, skeletond, skeletondd);
                    // }
                    // auto delay_time = std::chrono::steady_clock::now() - collision_time;
                    // loop_start += delay_time;
                    // next_time += delay_time;

                    continue;
                }

                // cnt += period_ms;
            }
            else {
                break;
            }

            

            auto time2 = std::chrono::steady_clock::now() - time1;
            double task_duration = std::round(std::chrono::duration<double>(time2).count() * 1000);

            // std::cout << "task_duration [ms] = " << task_duration << std::endl;

            next_time += std::chrono::milliseconds(period_ms);
            std::this_thread::sleep_until(next_time);

            
        }

        std::cout << "++++++++++++++++++++++++++++++++" << std::endl;
        std::cout << "+++++ Trajectory completed +++++" << std::endl;
        std::cout << "++++++++++++++++++++++++++++++++" << std::endl;

        next_time = std::chrono::steady_clock::now();
        loop_start = std::chrono::steady_clock::now();
        cnt = 0;
    }

    for (auto &transmitter : transmitters) {
        transmitter->shutdown();
    }

    return 0;
}


/*int test(std::string c_dir="") {

    const std::string urdf_path = c_dir + "/src/urdf/panda.urdf";
    RobotModel robot(urdf_path);

    const double dt = 0.005;

    stopping_time = 0.25;

    q_real = Eigen::VectorXd(7);
    q_real << 0.6152, 0.5977, -0.1659, -1.4862, 0.0936, 2.0707, 0.4532;

    qd_real = Eigen::VectorXd(7);
    qd_real << 0.4543, -0.0319, -0.1200, 0.0271, 0.0409, -0.0969, 0.2976;

    std::array<Eigen::Vector3d, 2> p_r;
    p_r[1] << 0.6207, 0.3424, 0.3264;

    std::array<Eigen::Vector3d, 2> pd_r;
    pd_r[1] << -0.1450, 0.2656, 0.0321;

    std::array<Eigen::VectorXd, 2> q_r;
    q_r[1] = Eigen::VectorXd(7);
    q_r[1] << 0.6282, 0.6011, -0.1573, -1.4823, 0.0947, 2.0725, 0.4552;

    Eigen::Vector3d skeleton;
    skeleton << 0.6120, 0.6120, 0.3000;

    Eigen::Vector3d skeletond;
    skeletond << 0.5988, 0.5988, 0;

    double velocity_term = 0.2290;
    
    double Qpj = 70;
    
    double Qpt = 1;
    
    double Qv = 0.08;




    // std::cout << "dt = " << dt << std::endl; 
    // std::cout << "stopping_time = " << stopping_time << std::endl; 
    // std::cout << "q_real = " << q_real << std::endl; 
    // std::cout << "qd_real = " << qd_real << std::endl; 
    // std::cout << "p_r[1] = " << p_r[1] << std::endl; 
    // std::cout << "pd_r[1] = " << pd_r[1] << std::endl; 
    // std::cout << "q_r[1] = " << q_r[1] << std::endl; 
    // std::cout << "skeleton = " << skeleton << std::endl; 
    // std::cout << "skeletond = " << skeletond << std::endl; 
    // std::cout << "velocity_term = " << velocity_term << std::endl; 
    // std::cout << "Qpj = " << Qpj << std::endl; 
    // std::cout << "Qpt = " << Qpt << std::endl; 
    // std::cout << "Qv = " << Qv << std::endl; 

    SSMPFLResult res = SSMPFL(robot, dt, stopping_time, q_real, qd_real, p_r[1], pd_r[1], q_r[1], skeleton, skeletond, velocity_term, Qpj, Qpt, Qv);
    
    return 0;
}*/



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

    // test(path);
    // return 0;

    if (execute_task(n_traj, path)) return 1;
    else printf("Exiting cleanly...\n");
    
    return 0;
}