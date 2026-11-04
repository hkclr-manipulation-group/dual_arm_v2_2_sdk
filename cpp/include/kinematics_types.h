#ifndef DUAL_ARM_V2_2_KINEMATICS_TYPES_H
#define DUAL_ARM_V2_2_KINEMATICS_TYPES_H

#include <array>
#include <cstdint>

namespace dual_arm_v2_2_sdk {

constexpr int kArmDof = 7;
constexpr int kToolPoseFloats = 7;

using Joint7Rad = std::array<float, kArmDof>;
using ToolPose7 = std::array<float, kToolPoseFloats>;

enum class MotionRetCode : int {
    SUCCESS = 0,
    NOT_INITIALIZED = 1,
    INVALID_ARGUMENT = 2,
    IK_FAILED = 3,
    IK_NEAR_SINGULAR = 4,
    PATH_BLOCKED = 5,
    JOINT_LIMIT = 6,
    JOINT_STEP_TOO_LARGE = 7,
    CONTROLLER_BUSY = 8,
    NO_STATE = 9,
};

enum class CartesianSide : uint8_t { LEFT = 0, RIGHT = 1 };

enum class AsyncMotionState : uint8_t {
    IDLE = 0,
    STREAMING = 1,
    TASK_MOVING = 2,
    TASK_HOLD = 3,
    ERROR = 4,
};

struct SingularityPolicy {
    float sigma_min_threshold = 0.02F;
    int max_ik_retries = 4;
    float max_delta_q_rad = 0.12F;
    float max_joint_jump_rad = 1.2F;
    bool reject_entire_line = true;
    float line_ds_min = 0.02F;
};

struct AsyncStreamOptions {
    int coalesce_hz = 100;
    CartesianSide cartesian_arm = CartesianSide::RIGHT;
    SingularityPolicy singularity{};
    float tool_pos_tolerance_m = 0.0005F;
    float tool_ori_tolerance_rad = 0.00872665F;
};

struct ToolLineSpec {
    ToolPose7 start{};  // ignored when start_is_current == true
    ToolPose7 end{};
    uint16_t speed_ratio = 15;
    float acc_time_s = 4.0F;
    bool start_is_current = true;
};

}  // namespace dual_arm_v2_2_sdk

#endif
