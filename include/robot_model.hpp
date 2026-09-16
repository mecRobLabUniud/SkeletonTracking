// robot_model.hpp
#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class RobotModel {
    public:
    // Loads the model from a URDF file. `root_joint` can be left null
    // (fixed base, e.g. an arm bolted to a table like the Panda).
    explicit RobotModel(const std::string& urdf_path);

    // --- Forward kinematics -------------------------------------------------

    // Updates internal kinematic data for configuration q.
    // Call this before GetJointPose() / ComputeJacobian() if you want them
    // to reflect a new q; ComputeFK() is otherwise called automatically
    // by the pose/Jacobian methods that take q directly.
    void ComputeFK(const Eigen::VectorXd& q) const;

    // Pose of a named frame/joint (as defined in the URDF) at configuration q.
    Eigen::Isometry3d GetJointPose(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    // Pose of a named frame/joint using the last configuration passed to
    // ComputeFK(). Cheaper if you need several frames at the same q.
    Eigen::Isometry3d GetJointPose(const std::string& frame_name) const;

    // --- Jacobian ------------------------------------------------------------

    // 6xN geometric Jacobian of `frame_name` at configuration q, expressed
    // in the LOCAL_WORLD_ALIGNED frame (linear velocity in world axes,
    // computed at the frame origin — standard convention for SSM/PFL work).
    Eigen::MatrixXd ComputeJacobian(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    Eigen::MatrixXd ComputeDerivativeJacobian(const std::string& frame_name,
                                    const Eigen::VectorXd& q) const;

    private:
    pinocchio::FrameIndex GetFrameIndexOrThrow(
        const std::string& frame_name) const;

    pinocchio::Model model_;
    mutable pinocchio::Data data_;
};