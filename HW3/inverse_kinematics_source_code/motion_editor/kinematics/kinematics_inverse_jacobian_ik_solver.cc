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
{
    math::Vector6dColl_t bodyPos6d = original_whole_body_joint_pos6d;

    int JacobianColNum = 0;
    int32_t tempBoneIdx = end_bone_idx;

    // Compute Jacobian column number
    while (tempBoneIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx) {
        JacobianColNum += skeleton_->bone_ptr(tempBoneIdx)->dof;
        tempBoneIdx = skeleton_->bone_ptr(tempBoneIdx)->parent->idx;
    }

    for (int iteration = 0; iteration < max_iteration_num_; iteration++) {
        PoseColl_t temp;
        temp = fk_solver_->ComputeSkeletonPose(bodyPos6d);
        math::Vector3d_t V = target_pos - temp[end_bone_idx].end_pos();

        if (V.norm() > distance_epsilon_) {
            tempBoneIdx = end_bone_idx;
            int dofInc = 0;
            math::MatrixN_t Jacobian(3, JacobianColNum);

            // Compute Jacobian
            while (tempBoneIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx) {
                // (p - ri)
                math::Vector3d_t r = temp[end_bone_idx].end_pos() - temp[tempBoneIdx].start_pos();
                for (int colIdx = 0; colIdx < skeleton_->bone_ptr(tempBoneIdx)->dof;) {
                    if (skeleton_->bone_ptr(tempBoneIdx)->dofx) {
                        Jacobian.col(colIdx + dofInc) = temp[tempBoneIdx].rotation().col(0).cross(r);
                        colIdx++;
                    }
                    else if (skeleton_->bone_ptr(tempBoneIdx)->dofy) {
                        Jacobian.col(colIdx + dofInc) = temp[tempBoneIdx].rotation().col(1).cross(r);
                        colIdx++;
                    }
                    else if (skeleton_->bone_ptr(tempBoneIdx)->dofz) {
                        Jacobian.col(colIdx + dofInc) = temp[tempBoneIdx].rotation().col(2).cross(r);
                        colIdx++;
                    }
                }
                dofInc += skeleton_->bone_ptr(tempBoneIdx)->dof;
                tempBoneIdx = skeleton_->bone_ptr(tempBoneIdx)->parent->idx;
            }

            tempBoneIdx = end_bone_idx;
            dofInc = 0;

            math::VectorNd_t deltaSita(Jacobian.cols());
            deltaSita = linear_system_solver_->Solve(Jacobian, V);
            while (tempBoneIdx != skeleton_->bone_ptr(start_bone_idx)->parent->idx){
                math::Vector3d_t angularVector = bodyPos6d[tempBoneIdx].angular_vector();
                for (int colIdx = 0; colIdx < skeleton_->bone_ptr(tempBoneIdx)->dof;) {
                    if (skeleton_->bone_ptr(tempBoneIdx)->dofx) {
                        angularVector[0] += deltaSita[0 + dofInc] * step_;
                        colIdx++;
                    }
                    else if (skeleton_->bone_ptr(tempBoneIdx)->dofy) {
                        angularVector[1] += deltaSita[1 + dofInc] * step_;
                        colIdx++;
                    }
                    else if (skeleton_->bone_ptr(tempBoneIdx)->dofz){
                        angularVector[2] += deltaSita[2 + dofInc] * step_;
                        colIdx++;
                    }
                }
                bodyPos6d[tempBoneIdx].set_angular_vector(angularVector);
                dofInc += skeleton_->bone_ptr(tempBoneIdx)->dof;
                tempBoneIdx = skeleton_->bone_ptr(tempBoneIdx)->parent->idx;
            }
        }
        else {
            return bodyPos6d;
        }
    }
    return bodyPos6d;
}

} // namespace kinematics {

