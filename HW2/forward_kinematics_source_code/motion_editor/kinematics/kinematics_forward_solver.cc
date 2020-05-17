#include "kinematics_forward_solver.h"
#include <algorithm>
#include "math_utils.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "helper_forward_kinematics.h"
#include "kinematics_artic_idx.h"
#include "kinematics_pose.h"

namespace kinematics {

// public func.

ForwardSolver::ForwardSolver()
    :skeleton_(nullptr),
    motion_(nullptr),
    artic_path_(new ArticIdxColl_t),
    helper_fk_(new helper::ForwardKinematics)
{
}

ForwardSolver::~ForwardSolver()
{
}

std::shared_ptr<acclaim::Skeleton> ForwardSolver::skeleton() const
{
    return skeleton_;
}

std::shared_ptr<acclaim::Motion> ForwardSolver::motion() const
{
    return motion_;
}

void ForwardSolver::set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_ = skeleton;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::set_motion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_ = motion;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::ConstructArticPath()
{
    helper_fk_->ConstructArticPath();
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const int32_t frame_idx)
{
    return this->ComputeSkeletonPose(motion_->joint_spatial_pos(frame_idx));
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const math::Vector6dColl_t &joint_spatial_pos)
{
    PoseColl_t pose_collection;
    Pose root;
    root.set_start_pos(joint_spatial_pos.at(0).linear_vector());
    root.set_end_pos(joint_spatial_pos.at(0).linear_vector());
    root.set_rotation(math::ComputeRotMatXyz(
        math::ToRadian(skeleton()->bone_ptr(0)->axis)
        )
    );
    pose_collection.push_back(root);
    
    for (int boneidx = 1; boneidx < skeleton()->bone_num(); boneidx++) {
        acclaim::Bone current_bone = *skeleton()->bone_ptr(boneidx);

        math::RotMat3d_t R_asf, R_amc, R_i_0, R_root;
        R_i_0.setIdentity();
        R_root = math::ComputeRotMatXyz(
            math::ToRadian(joint_spatial_pos.at(0).angular_vector())
        );
        
        while (current_bone.idx != 0) {
            R_asf = math::ToRotMat(current_bone.rot_parent_current).transpose();
            R_amc = math::ComputeRotMatXyz(
                math::ToRadian(
                    joint_spatial_pos.at(current_bone.idx).angular_vector()
                )
            );
            R_i_0 = R_asf * R_amc * R_i_0;

            current_bone = *current_bone.parent;
        }
        R_i_0 = R_root * R_i_0;

        math::Vector3d_t V_i = skeleton()->bone_ptr(boneidx)->dir * skeleton()->bone_ptr(boneidx)->length;
        math::Vector3d_t T_i = pose_collection.at(skeleton()->bone_ptr(boneidx)->parent->idx).end_pos();
        math::Vector3d_t T_i_plus_1 = R_i_0 * V_i + T_i;

        Pose temp;
        temp.set_start_pos(T_i);
        temp.set_end_pos(T_i_plus_1);
        temp.set_rotation(math::ComputeRotMatXyz(
            math::ToRadian(skeleton()->bone_ptr(boneidx)->axis)
            )
        );
        
        pose_collection.push_back(temp);
    }
       
    return pose_collection;
}


// protected func.

// private func.

} // namespace kinematics {
