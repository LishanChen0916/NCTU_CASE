#include "kinematics_time_warper.h"
#include <utility>
#include "boost/numeric/conversion/cast.hpp"
#include "math_utils.h"

namespace kinematics {

// public func.

TimeWarper::TimeWarper()
    :original_motion_sequence_(new math::SpatialTemporalVector6d_t),
    hard_constraint_coll_(new TimeWarpHardConstraintColl_t),
    time_step_(double{0.0}),
    min_time_step_(double{0.0}),
    max_time_step_(double{0.0})
{
}

TimeWarper::~TimeWarper()
{
}

double TimeWarper::time_step() const
{
    return time_step_;
}

double TimeWarper::min_time_step() const
{
    return min_time_step_;
}

double TimeWarper::max_time_step() const
{
    return max_time_step_;
}

void TimeWarper::Configure(
        const math::SpatialTemporalVector6d_t &original_motion_sequence,
        const double time_step,
        const double min_time_step,
        const double max_time_step
        )
{
    *original_motion_sequence_ = original_motion_sequence;
    time_step_ = time_step;
    min_time_step_ = min_time_step;
    max_time_step_ = max_time_step;
}

math::SpatialTemporalVector6d_t TimeWarper::ComputeWarpedMotion(
        const TimeWarpHardConstraintColl_t &hard_constraint_coll
        )
{
    *hard_constraint_coll_ = hard_constraint_coll;
    math::SpatialTemporalVector6d_t whole_motion_sequence_ = *original_motion_sequence_;
    float frame_per_second = hard_constraint_coll_->at(2).frame_idx / hard_constraint_coll_->at(2).play_second;
    int32_t warped_desired_catch_ball_frame_idx = frame_per_second * hard_constraint_coll_->at(1).play_second;
    int32_t desired_catch_ball_frame_idx = hard_constraint_coll_->at(1).frame_idx;

    float frame_step_before_catch_ball = (float)warped_desired_catch_ball_frame_idx / (float)desired_catch_ball_frame_idx;
    float frame_step_after_catch_ball = (float)(hard_constraint_coll_->at(2).frame_idx - warped_desired_catch_ball_frame_idx) /
        (float)(hard_constraint_coll_->at(2).frame_idx - desired_catch_ball_frame_idx);
    
    for (int frameidx = 0; frameidx < original_motion_sequence_->temporal_size(); frameidx++) {
        for (int boneidx = 0; boneidx < original_motion_sequence_->spatial_size(); boneidx++) {
            if (frameidx <= desired_catch_ball_frame_idx) {
                float slerp_frame_idx = frame_step_before_catch_ball * frameidx;
                int32_t prev_frame_idx = floorf(slerp_frame_idx);
                int32_t next_frame_idx = ceilf(slerp_frame_idx);

                math::Quaternion_t prev_Q = math::ComputeQuaternionXyz(
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).x(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).y(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).z()
                );

                math::Quaternion_t next_Q = math::ComputeQuaternionXyz(
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).x(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).y(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).z()
                );

                double slerpRatio = slerp_frame_idx - prev_frame_idx;
                math::Vector6d_t temp = original_motion_sequence_->element(boneidx, frameidx);
                temp.set_angular_vector(math::ToDegree(
                    math::ComputeEulerAngleXyz(
                        math::ComputeRotMat(
                            math::Slerp(prev_Q, next_Q, slerpRatio)
                        )
                    )
                ));
                whole_motion_sequence_.set_element(boneidx, frameidx, temp);
            }

            else {
                float slerp_frame_idx = warped_desired_catch_ball_frame_idx + 
                    frame_step_after_catch_ball * (frameidx - desired_catch_ball_frame_idx);
                int32_t prev_frame_idx = floorf(slerp_frame_idx);
                int32_t next_frame_idx = ceilf(slerp_frame_idx);

                math::Quaternion_t prev_Q = math::ComputeQuaternionXyz(
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).x(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).y(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, prev_frame_idx).angular_vector()).z()
                );

                math::Quaternion_t next_Q = math::ComputeQuaternionXyz(
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).x(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).y(),
                    math::ToRadian(original_motion_sequence_->element(boneidx, next_frame_idx).angular_vector()).z()
                );

                double slerpRatio = slerp_frame_idx - prev_frame_idx;
                math::Vector6d_t temp = original_motion_sequence_->element(boneidx, frameidx);
                temp.set_angular_vector(math::ToDegree(
                    math::ComputeEulerAngleXyz(
                        math::ComputeRotMat(
                            math::Slerp(prev_Q, next_Q, slerpRatio)
                        )
                    )
                ));
                whole_motion_sequence_.set_element(boneidx, frameidx, temp);
            }
        }
    }

    return whole_motion_sequence_;
}


// protected func.

// private func.

} // namespace kinematics {
