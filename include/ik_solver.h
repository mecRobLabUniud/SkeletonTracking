/*
░IK_SOLVER░
░IK_SOLVER░
░IK_SOLVER░

ik_solver.h
*/

#pragma once
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/explog.hpp>


struct IKResult {
    Eigen::VectorXd q;   // joint configuration that best matches the target
};

pinocchio::SE3 buildTarget(const Eigen::Vector3d& pos);

IKResult ikine_LM(
    const pinocchio::Model&     model,
    pinocchio::Data&            data,
    const pinocchio::SE3&       target,
    pinocchio::FrameIndex       frame_id,
    const Eigen::VectorXd&      q0,
    const Eigen::VectorXd*      q_ref       = nullptr,
    double                      w_secondary = 5e-2,
    double                      max_step    = 0.2
);