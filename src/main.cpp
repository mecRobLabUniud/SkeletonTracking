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
//   * Plots are replaced by a CSV dump (paths.csv) you can plot with anything.

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




// ───────────────────────────────────────────────────────────────────────────
const double freq = 60.0;
const double dt = 1.0 / freq;
const double stopping_time = 0.25;
const double pause_after_collision = 3.75;
const double velocity_PFL = 0.4;
const double HR_clearance = 0.1;
const double Qpj = 70.0;
const double Qpt = 1.0;
const double Qv = 0.08;
std::atomic<bool> running{true};


void signal_handler(int signum) {
    (void)signum;
    running = false;
}


// Sends one 3D dataset to an open gnuplot pipe, terminated with "e".
template <typename GetPoint>
static void SendXYZ(FILE* gp, int n, GetPoint get) {
    for (int k = 0; k < n; ++k) {
        const Eigen::Vector3d p = get(k);
        std::fprintf(gp, "%f %f %f\n", p.x(), p.y(), p.z());
    }
    std::fprintf(gp, "e\n");
}

void PlotPaths(const std::vector<Eigen::Vector3d>& p_real, const Eigen::MatrixXd& p_int,
                const Eigen::MatrixXd& pf, const Eigen::MatrixXd& ps) {
    FILE* gp = popen("gnuplot -persist", "w");
    if (!gp) { std::cerr << "gnuplot not found\n"; return; }

    std::fprintf(gp,
            "set title 'Path of both manipulator and operator'\n"
            "set xlabel 'x [m]'; set ylabel 'y [m]'; set zlabel 'z [m]'\n"
            "set view equal xyz\n"
            "set view 70, 125\n"          // <-- elevation, azimuth
            "set grid\n"
            "splot '-' with lines lc rgb '#0072BD' title 'robot (real)', \\\n"
            "      '-' with lines lc rgb 'red'     title 'human', \\\n"
            "      '-' with lines lc rgb 'green'   title 'nominal forward', \\\n"
            "      '-' with lines lc rgb 'green'   title 'nominal backward'\n");

    SendXYZ(gp, static_cast<int>(p_real.size()), [&](int k) { return p_real[k]; });
    SendXYZ(gp, static_cast<int>(p_int.cols()),  [&](int k) { return Eigen::Vector3d(p_int.col(k)); });
    SendXYZ(gp, static_cast<int>(pf.cols()),     [&](int k) { return Eigen::Vector3d(pf.col(k)); });
    SendXYZ(gp, static_cast<int>(ps.cols()),     [&](int k) { return Eigen::Vector3d(ps.col(k)); });

    pclose(gp);
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

static void WriteCsv(const std::string& path, const std::vector<Eigen::Vector3d>& v) {
    std::ofstream f(path);
    f << "x,y,z\n";
    for (const auto& p : v) f << p.x() << "," << p.y() << "," << p.z() << "\n";
    }
    static void WriteCsv(const std::string& path, const Eigen::MatrixXd& m) {
    std::ofstream f(path);
    f << "x,y,z\n";
    for (int k = 0; k < m.cols(); ++k)
        f << m(0, k) << "," << m(1, k) << "," << m(2, k) << "\n";
}




/*Trajectory_new load_p2p_trajectory(RobotModel robot, double time_final, double dt, int N, int M) {
    const std::string ee = "panda_link8";

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
}*/


// ─────────────────────────────────────────────────────────────────────────────
// Load trajectory
// ─────────────────────────────────────────────────────────────────────────────
Trajectory load_p2p_trajectory(RobotModel robot, double time_final, double dt, int N) {
    const std::string ee = "panda_link8";

    std::vector<Eigen::Vector3d> robot_p;
    robot_p.push_back(Eigen::Vector3d{0.7, 0.1, 0.3});
    robot_p.push_back(Eigen::Vector3d{0.1, 0.7, 0.4});
    robot_p.push_back(Eigen::Vector3d{0.7, 0.1, 0.3});

    Eigen::VectorXd q_base(7);
    q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;

    std::vector<Eigen::VectorXd> robot_q;
    robot_q.reserve(robot_p.size());
    for (int i=0; i<robot_p.size(); i++) {
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R(1, 1) = -1.0;
        R(2, 2) = -1.0;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;  T.translation() = robot_p[i];

        Eigen::VectorXd q;
        if (!robot.ComputeIK(ee, T, q_base, &q)) throw std::runtime_error("IK failed for start pose");
        robot_q.push_back(q);
    }

    auto build_traj = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                          std::vector<double>* vnorm, Trajectory& out, double& t) {
        Traj tr = QuinticPolyTraj(a, b, time_final, dt, N);

        for (int k = 0; k < N; ++k) {
            const Eigen::VectorXd q = tr.p.col(k);
            const Eigen::VectorXd qd = tr.v.col(k);
            const Eigen::VectorXd qdd = tr.a.col(k);

            const Eigen::Matrix<double, 6, 1> v = robot.ComputeJacobian(ee, q) * qd;

            out.q.push_back(q);
            out.qd.push_back(qd);
            out.qdd.push_back(qdd);
            out.p.push_back(robot.GetJointPose(ee, q).translation());
            out.pd.push_back(v.head<3>());
            out.t.push_back(dt*t++);
            if (vnorm) vnorm->push_back(v.norm());
        }

        return out;
    };

    std::vector<double> v_norm;
    Trajectory traj;
    traj.q.reserve(N*(robot_q.size() - 1));
    traj.qd.reserve(N*(robot_q.size() - 1));
    traj.qdd.reserve(N*(robot_q.size() - 1));
    traj.p.reserve(N*(robot_q.size() - 1));
    traj.pd.reserve(N*(robot_q.size() - 1));
    traj.t.reserve(N*(robot_q.size() - 1));

    double t = 0;
    build_traj(robot_q[0], robot_q[1], &v_norm, traj, t);
    build_traj(robot_q[1], robot_q[2], &v_norm, traj, t);
    build_traj(robot_q[0], robot_q[1], &v_norm, traj, t);
    build_traj(robot_q[1], robot_q[2], &v_norm, traj, t);
    
    double v_max = *std::max_element(v_norm.begin(), v_norm.end());
    double v_rms = 0;
    for (double x : v_norm) v_rms += x * x;
    v_rms = std::sqrt(v_rms / v_norm.size());
    std::cout << "v_max = " << v_max << "\nv_norm_base(rms) = " << v_rms << "\n";

    return traj;
}



/*
// ─────────────────────────────────────────────────────────────────────────────
// Load trajectory
// ─────────────────────────────────────────────────────────────────────────────
std::optional<Trajectory> load_trajectory(RobotModel robot, int n_traj, std::string c_dir, double t_start = 0.0, Eigen::VectorXd q_start = {}) {
    std::string trajectory_path = c_dir + "src/trajectories/test" + std::to_string(n_traj) + "/";
    std::ifstream f(trajectory_path);
    if (!f) {
        std::cerr << "Error: cannot open '" << trajectory_path << "'" << std::endl;
        return std::nullopt;
    }

    const std::string ee = "panda_link8";

    Eigen::VectorXd q_base(7);
    q_base << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;



    std::vector<std::array<double, 3>> traj_low_x = load_trajectory_CSV(trajectory_path + "q_ref.csv");
    std::vector<double> t_low = load_timestamps_CSV(trajectory_path + "t_ref.csv");

    std::vector<std::array<double, 7>> traj_low;
    for (auto v : traj_low_x) {
        Eigen::Vector3d x = Eigen::Map<const Eigen::Vector3d>(v.data()); 
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        R(1, 1) = -1.0;
        R(2, 2) = -1.0;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;  T.translation() = x;

        std::cout << "x = " << x << "\n";

        Eigen::VectorXd q(7);
        if (!robot.ComputeIK(ee, T, q_base, &q)) throw std::runtime_error("IK failed for waypoint pose");

        std::cout << "q = " << q << "\n";

        std::array<double, 7> arr;
        Eigen::Map<Eigen::VectorXd>(arr.data(), 7) = q;
        traj_low.push_back(arr);
    }

    
    

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



    Trajectory traj_high = interpolateQuintic(traj_low, t_low);



    for (auto q : traj_high.q) {
        Eigen::MatrixXd J = robot.ComputeJacobian("panda_link8", q).topRows(3);
        Eigen::Vector3d p = robot.GetJointPose("panda_link8", q).translation().transpose();
        traj_high.p.push_back(std::move(p));
        traj_high.pd.push_back(std::move(J * q));
    }
    

    // Trajectory traj_high = interpolate_to_1kHz_full(traj_low, t_low);

    // save_trajectory_CSV(trajectory_path + "q.csv",  traj_high.q);
    // save_trajectory_CSV(trajectory_path + "qd.csv",  traj_high.qd);
    // save_trajectory_CSV(trajectory_path + "qdd.csv",  traj_high.qdd);

    return traj_high;
}
*/


// ─────────────────────────────────────────────────────────────────────────────
// Main loop implementing chosen strategy
// ─────────────────────────────────────────────────────────────────────────────
// int task_engine() {}




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
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start).count() <= 3.2) {;} 
        
    // ── Load trajectory ──────────────────────────────────────────────────────────
    // auto traj = load_trajectory(robot, n_traj, c_dir);
    // if (!traj) return 1;
    
    
    double time_final = 5.0;    

    const int N = static_cast<int>(std::lround(time_final / dt)) + 1;  // 1001
    const int Nc = static_cast<int>(std::lround(time_final / dt)) + 1;

    Trajectory nominal_traj = load_p2p_trajectory(robot, time_final, dt, N);

    const int n_steps = static_cast<int>(freq*time_final*nominal_traj.t.size()/N);

    // ── PFL & SSM & Escape traj simulation ───────────────────────────────────────
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
    Eigen::Vector3d ro_prev;
    Eigen::Vector3d vo_prev;

    int keypoint_index = 8;

    const int period_ms = static_cast<int>(dt*1000.0);
    auto next_time = std::chrono::steady_clock::now();

    for (int i = 1; i <= n_steps; ++i) {
        const int k = i - 1;  // index of the latest state (MATLAB column i)

        if (!collision) {
        const int rr = r; // std::min(r, M - 1);

        skeleton = json_to_keypoints(transmitters[0]->receive_data()[0]);


            if (std::isnan(skeleton[keypoint_index][0])) {
                ro = ro_prev;
                vo = vo_prev;
            } else {
                ro = skeleton[keypoint_index];
                vo = (skeleton[keypoint_index] - skeleton_prev[keypoint_index])/dt;
            }


        const double delta =
            -(-(HR_clearance + vo.norm() * stopping_time) / stopping_time + velocity_PFL) *
            stopping_time;

        SSMPFLResult res = SSMPFL(robot, dt, stopping_time, real_traj.q[k], real_traj.qd[k],
                                    nominal_traj.p[rr], nominal_traj.pd[rr], nominal_traj.q[rr],
                                    ro, vo, delta, Qpj, Qpt, Qv);
        real_traj.qdd.push_back(res.qdd_next);
        real_traj.qd.push_back(res.qd_next);
        real_traj.q.push_back(res.q_next);
        real_traj.p.push_back(res.p_next);
        real_traj.pd.push_back(res.pd_next);
        // flag.push_back(res.exitflag ? 1 : 0);
        ++r;



        std::vector<nlohmann::json> payload;
        payload.push_back(std::vector<std::array<double, 3>>{{0, 0, 0}});
        payload.push_back(std::vector<double>(res.q_next.data(), res.q_next.data() + res.q_next.size()));
        payload.push_back(std::vector<int>{});
        transmitters[1]->send_data(payload);

        payload.clear();
        payload.push_back(std::array<double, 3>{{ro[0], ro[1], ro[2]}});
        payload.push_back(std::array<double, 3>{{res.p_next[0], res.p_next[1], res.p_next[2]}});
        transmitters[2]->send_data(payload);

        Eigen::Vector3d p_r;
        p_r = robot.GetJointPose("panda_link8", nominal_traj.q[rr]).translation().transpose();

        payload.clear();
        payload.push_back(std::vector<double>(res.p_next.data(), res.p_next.data() + res.p_next.size()));
        payload.push_back(std::vector<double>(p_r.data(), p_r.data() + p_r.size()));
        transmitters[3]->send_data(payload);


        ro_prev = ro;
        vo_prev = vo;
        skeleton_prev = skeleton;


        if (res.exitflag) {
            if (!CollisionFree(robot, real_traj.q[k], ro, HR_clearance)) {
              collision = true;
            }
            // if ((real_traj.p[k] - nominal_traj.p[N - 1]).norm() <= 0.01) {
            //     r = 1;
            // }

        } else {
            collision = true;

        }
        } else {
        real_traj.qdd.push_back(Eigen::VectorXd::Zero(7));
        real_traj.qd.push_back(Eigen::VectorXd::Zero(7));
        // NOTE: faithful to MATLAB `real_traj.q(:,end-1)` / `real_traj.p(:,end-1)`, which
        // repeats the sample *before* the latest one. Use [k] instead if you
        // actually intend the robot to hold its current pose during the pause.
        const int prev = k > 0 ? k - 1 : k;
        real_traj.q.push_back(real_traj.q[prev]);
        real_traj.p.push_back(real_traj.p[prev]);
        real_traj.pd.push_back(Eigen::Vector3d::Zero());
        // if (collision_counter > pause_after_collision * freq) collision = false;
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