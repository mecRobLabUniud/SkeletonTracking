#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>


// ─────────────────────────────────────────────────────────────────────────────
// Pinocchio wrapper providing forward kinematics, frame poses, Jacobians and
// damped-least-squares inverse kinematics for a URDF robot model
// ─────────────────────────────────────────────────────────────────────────────
class RobotModel {
    public:
    // ─────────────────────────────────────────────────────────────────────────────
    // Load the model from a URDF file (fixed base)
    // ─────────────────────────────────────────────────────────────────────────────
    explicit RobotModel(const std::string& urdf_path);

    // ─────────────────────────────────────────────────────────────────────────────
    // Update internal kinematic data for configuration q
    // ─────────────────────────────────────────────────────────────────────────────
    void ComputeFK(const Eigen::VectorXd& q) const;

    // ─────────────────────────────────────────────────────────────────────────────
    // Pose of a named frame at configuration q
    // ─────────────────────────────────────────────────────────────────────────────
    Eigen::Isometry3d GetJointPose(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    // ─────────────────────────────────────────────────────────────────────────────
    // Pose of a named frame using the last configuration passed to ComputeFK
    // ─────────────────────────────────────────────────────────────────────────────
    Eigen::Isometry3d GetJointPose(const std::string& frame_name) const;

    // ─────────────────────────────────────────────────────────────────────────────
    // Geometric Jacobian of a named frame at configuration q
    // ─────────────────────────────────────────────────────────────────────────────
    Eigen::MatrixXd ComputeJacobian(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    // ─────────────────────────────────────────────────────────────────────────────
    // Finite-difference derivative of the frame Jacobian at configuration q
    // ─────────────────────────────────────────────────────────────────────────────
    Eigen::MatrixXd ComputeDerivativeJacobian(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    // ─────────────────────────────────────────────────────────────────────────────
    // Damped-least-squares inverse kinematics to reach a target pose
    // ─────────────────────────────────────────────────────────────────────────────
    bool ComputeIK(const std::string& frame_name,
                    const Eigen::Isometry3d& target_pose,
                    const Eigen::VectorXd& q_init,
                    Eigen::VectorXd* q_result,
                    double eps = 1e-4,
                    int max_iters = 1000,
                    double damping = 1e-6) const;

    private:
    // ─────────────────────────────────────────────────────────────────────────────
    // Resolve a frame name to its Pinocchio index, throwing if absent
    // ─────────────────────────────────────────────────────────────────────────────
    pinocchio::FrameIndex GetFrameIndexOrThrow(
        const std::string& frame_name) const;

    pinocchio::Model model_;
    mutable pinocchio::Data data_;
};
