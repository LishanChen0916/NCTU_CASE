#include "kinematics_inverse_jacobian_ik_solver.h"
#include <limits>
#include "console_log.h"
#include "math_utils.h"
#include "math_linear_system_solver.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "kinematics_forward_solver.h"
#include "kinematics_pose.h"

namespace kinematics {

InverseJacobianIkSolver::InverseJacobianIkSolver()
    :skeleton_(nullptr),
    fk_solver_(new ForwardSolver),
    step_(double{0.0}),
    distance_epsilon_(double{0.0}),
    max_iteration_num_(0),
    linear_system_solver_(nullptr)
{
}

InverseJacobianIkSolver::~InverseJacobianIkSolver()
{
}

void InverseJacobianIkSolver::Configure(
        const std::shared_ptr<acclaim::Skeleton> &skeleton,
        const std::shared_ptr<math::LinearSystemSolver> &linear_system_solver,
        const double step,
        const double distance_epsilon,
        const int32_t max_iteration_num
        )
{
    skeleton_ = skeleton;
    fk_solver_->set_skeleton(skeleton_);
    fk_solver_->ConstructArticPath();

    linear_system_solver_ = linear_system_solver;

    step_ = step;
    distance_epsilon_ = distance_epsilon;
    max_iteration_num_ = max_iteration_num;
}

math::Vector6dColl_t InverseJacobianIkSolver::Solve(
        const math::Vector3d_t &target_pos,
        const int32_t start_bone_idx,
        const int32_t end_bone_idx,
        const math::Vector6dColl_t &original_whole_body_joint_pos6d
        )
{//TO DO
	return original_whole_body_joint_pos6d;
}

} // namespace kinematics {

