#include "ik_solver.h"

// --- helper: project a (possibly slightly non-orthonormal, due to floating
//     point drift) SE3's rotation back onto SO(3) via quaternion round-trip.
//     pinocchio::log3/log6 compute theta = acos((trace(R)-1)/2); if R has
//     drifted even a tiny bit, trace(R) can slip past -1 and acos() returns
//     NaN, which trips pinocchio's internal assertion. Round-tripping through
//     a normalized quaternion guarantees a valid rotation matrix and removes
//     that failure mode entirely. ---
static pinocchio::SE3 orthonormalize(const pinocchio::SE3& M)
{
    Eigen::Quaterniond quat(M.rotation());
    quat.normalize();
    return pinocchio::SE3(quat.toRotationMatrix(), M.translation());
}

// --- helper function ---
pinocchio::SE3 buildTarget(const Eigen::Vector3d& pos)
{
    // aggiunge la rotazione verso il basso all'obiettivo
    Eigen::Matrix3d R_yaw_pi;
    R_yaw_pi << 1,  0, 0,
                 0, -1, 0,
                 0,  0, -1;

    return pinocchio::SE3(R_yaw_pi, pos);
}

IKResult ikine_LM(
    const pinocchio::Model&  model,
    pinocchio::Data&         data,
    const pinocchio::SE3&    target,          // desired EE pose
    const pinocchio::FrameIndex frame_id,     // EE frame
    const Eigen::VectorXd&   q0,              // start configuration
    const Eigen::VectorXd*   q_ref,           // secondary-objective reference (nullptr -> q0)
    double                   w_secondary,     // weight of the "stay near q_ref" pull
    double                   max_step         // per-iteration cap on ||dq||_inf
)
{
    double  tol        = 1e-6;
    int     max_iter   = 100;
    double  lambda0    = 1e-3;                // initial damping
    const int NQ = model.nq;
    Eigen::VectorXd q = q0;
    const Eigen::VectorXd qr = q_ref ? *q_ref : q0;

    double lambda = lambda0;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // ── FK + frame update ──
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacement(model, data, frame_id);

        // ── Pose error in se(3) via log map ──
        const pinocchio::SE3 M_current = data.oMf[frame_id];
        const pinocchio::SE3 M_err     = orthonormalize(M_current.inverse() * target); // local error
        const pinocchio::Motion xi     = pinocchio::log6(M_err);       // 6D twist error

        Eigen::Matrix<double,6,1> err_vec = xi.toVector();             // [v; w]

        // ── Convergence check ──
        if (err_vec.norm() < tol) {
            return { q };
        }

        // ── Jacobian in local frame ──
        pinocchio::computeFrameJacobian(
            model, data, q, frame_id,
            pinocchio::LOCAL,   // same convention as the log6 error above
            data.J);            // 6 x NQ

        Eigen::MatrixXd J = data.J; // 6 x NQ

        // ── Primary task: damped least squares for the pose error ──
        // Δq_task = (JᵀJ + λI)⁻¹ Jᵀe   — this alone drives e -> 0 when the
        // target is reachable, with NO bias from the secondary objective.
        Eigen::MatrixXd JtJ = J.transpose() * J;                       // NQ x NQ
        Eigen::VectorXd dq_task = (JtJ + lambda * Eigen::MatrixXd::Identity(NQ, NQ))
                                  .ldlt()
                                  .solve(J.transpose() * err_vec);

        // ── Secondary task: pull toward q_ref, but ONLY within the nullspace
        //    of J (directions that don't move the end-effector at all).
        //    This is what makes the "stay near q_ref" bias-free: once e = 0,
        //    dq_task = 0 exactly, and this term can keep nudging redundant
        //    joints toward q_ref without ever reintroducing pose error. ──
        Eigen::MatrixXd J_pinv = J.transpose()
                                  * (J * J.transpose() + 1e-8 * Eigen::MatrixXd::Identity(6,6)).inverse();
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(NQ, NQ) - J_pinv * J;   // nullspace projector
        Eigen::VectorXd dq_secondary = w_secondary * (N * (qr - q));

        Eigen::VectorXd dq = dq_task + dq_secondary;

        // ── Cap step size so one Newton step can't jump the solver into an
        //    unrelated branch (this is what allows the base to "teleport" 180°
        //    within a single iteration even before hitting joint limits) ──
        double step_inf = dq.cwiseAbs().maxCoeff();
        if (step_inf > max_step)
            dq *= (max_step / step_inf);

        Eigen::VectorXd q_new = pinocchio::integrate(model, q, dq);

        // ── Clamp to joint limits ──
        q_new = q_new.cwiseMax(model.lowerPositionLimit)
                     .cwiseMin(model.upperPositionLimit);

        // ── LM damping update ──
        pinocchio::forwardKinematics(model, data, q_new);
        pinocchio::updateFramePlacement(model, data, frame_id);
        double new_err = pinocchio::log6(orthonormalize(data.oMf[frame_id].inverse() * target))
                         .toVector().norm();

        // Clamp lambda so it can neither collapse toward 0 (risking a huge,
        // near-singular step next iteration) nor blow up unbounded (freezing
        // the solver with vanishing steps while still far from converged) —
        // both are symptoms of the solver getting stuck oscillating instead
        // of making steady progress.
        const double lambda_min = 1e-8, lambda_max = 1e6;
        if (new_err < err_vec.norm())
            lambda *= 0.1;   // good step → reduce damping (more like Gauss-Newton)
        else
            lambda *= 10.0;  // bad step → increase damping (more like gradient descent)
        lambda = std::clamp(lambda, lambda_min, lambda_max);

        q = q_new;
    }

    // Return best effort if not converged
    return { q };
}