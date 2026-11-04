#ifndef DUAL_ARM_V2_2_ASYNC_DUAL_ARM_CONTROLLER_H
#define DUAL_ARM_V2_2_ASYNC_DUAL_ARM_CONTROLLER_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "control_api.h"
#include "kinematics.h"
#include "kinematics_types.h"

namespace dual_arm_v2_2_sdk {

class AsyncDualArmController {
public:
    explicit AsyncDualArmController(ControlApi& api,
                                    const std::string& config_prefix_path);
    ~AsyncDualArmController();

    AsyncDualArmController(const AsyncDualArmController&) = delete;
    AsyncDualArmController& operator=(const AsyncDualArmController&) = delete;

    MotionRetCode start(const AsyncStreamOptions& options);
    void stop(bool hold_last_targets = true);

    AsyncMotionState state() const;
    MotionRetCode last_error() const;

    /** Joint-role arm: smooth-step to target over duration_s (kDirect setpoints). */
    MotionRetCode set_joint_target(CartesianSide joint_side,
                                   const Joint7Rad& target_rad,
                                   float duration_s);

    MotionRetCode hold_joint_arm(CartesianSide joint_side);

    MotionRetCode move_tool_line(const ToolLineSpec& spec);
    MotionRetCode cancel_cartesian_motion();

    float cartesian_progress() const;
    float joint_move_progress(CartesianSide joint_side) const;

    MotionRetCode poll_reach(float* pos_err_m = nullptr,
                             float* ori_err_rad = nullptr) const;

    CartesianSide cartesian_arm() const { return cartesian_arm_; }

private:
    struct ActiveLine;
    struct ActiveJointRamp;

    ControlApi& api_;
    std::string config_prefix_;
    Kinematics kin_left_;
    Kinematics kin_right_;
    AsyncStreamOptions options_{};
    CartesianSide cartesian_arm_ = CartesianSide::RIGHT;

    mutable std::mutex mutex_;
    std::shared_ptr<ActiveLine> active_line_;
    std::shared_ptr<ActiveJointRamp> active_joint_ramp_;
    ToolPose7 task_goal_{};
    Joint7Rad task_q_cmd_{};
    Joint7Rad task_q_last_good_{};
    std::atomic<AsyncMotionState> state_{AsyncMotionState::IDLE};
    std::atomic<MotionRetCode> last_error_{MotionRetCode::SUCCESS};
    std::atomic<bool> planner_running_{false};
    std::thread planner_thread_;

    Kinematics& task_kinematics();
    const Kinematics& task_kinematics() const;
    Group task_group() const;
    Group joint_group(CartesianSide joint_side) const;

    MotionRetCode prescan_tool_line(const ToolPose7& start,
                                    const ToolPose7& end) const;
    bool fetch_task_seed(Joint7Rad& seed) const;
    bool fetch_tool_start(ToolPose7& start) const;
    void hold_both_from_feedback();
    void planner_loop();
    void apply_joint_ramp_step();
    void apply_task_step();
    void send_arm_joints(Group group, const Joint7Rad& q);
    MotionRetCode validate_joint_side(CartesianSide joint_side) const;
    CartesianSide expected_joint_side() const;
};

}  // namespace dual_arm_v2_2_sdk

#endif
