#ifndef DUAL_ARM_V2_2_CONTROL_API_C_H
#define DUAL_ARM_V2_2_CONTROL_API_C_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dual_arm_control_api dual_arm_control_api_t;

typedef enum {
    DUAL_ARM_GROUP_ALL = 1,
    DUAL_ARM_GROUP_LEFT_ARM = 2,
    DUAL_ARM_GROUP_RIGHT_ARM = 3,
    DUAL_ARM_GROUP_HEAD = 4,
    DUAL_ARM_GROUP_ELEVATOR = 5,
} dual_arm_group_t;

typedef enum {
    DUAL_ARM_RET_SUCCESS = 0,
    DUAL_ARM_RET_CONTROLLER_ERROR = 1,
    DUAL_ARM_RET_SEND_FAILED = -1,
    DUAL_ARM_RET_RECEIVE_FAILED = -2,
    DUAL_ARM_RET_PARSE_FAILED = -3,
    DUAL_ARM_RET_TIMEOUT = -4,
} dual_arm_ret_code_t;

typedef enum {
    DUAL_ARM_STATE_SHUTDOWN = 0,
    DUAL_ARM_STATE_OPERATIONAL = 1,
    DUAL_ARM_STATE_ERROR = 2,
} dual_arm_state_t;

typedef struct {
    uint16_t position;
    uint16_t force;
    uint16_t speed;
} dual_arm_gripper_state_t;

/** Create SDK client. Returns NULL on failure; check dual_arm_control_api_last_error(). */
dual_arm_control_api_t* dual_arm_control_api_create(const char* config_prefix_path);

/** Destroy SDK client created by dual_arm_control_api_create(). */
void dual_arm_control_api_destroy(dual_arm_control_api_t* api);

/** Last error message from create or API calls (thread-local static buffer). */
const char* dual_arm_control_api_last_error(const dual_arm_control_api_t* api);

const char* dual_arm_ret_code_name(dual_arm_ret_code_t code);
const char* dual_arm_group_name(dual_arm_group_t group);

dual_arm_ret_code_t dual_arm_set_group(dual_arm_control_api_t* api,
                                       dual_arm_group_t group, int enable);

dual_arm_ret_code_t dual_arm_get_group_state(dual_arm_control_api_t* api,
                                             dual_arm_group_t group,
                                             dual_arm_state_t* out_states,
                                             size_t out_capacity,
                                             size_t* out_count);

dual_arm_ret_code_t dual_arm_move_joint(dual_arm_control_api_t* api,
                                        dual_arm_group_t group,
                                        const float* joints, size_t joint_count,
                                        int follow, uint8_t trajectory_mode,
                                        uint16_t radio);

dual_arm_ret_code_t dual_arm_set_joint_target(dual_arm_control_api_t* api,
                                              dual_arm_group_t group,
                                              const float* joints,
                                              size_t joint_count);

dual_arm_ret_code_t dual_arm_flush_joint_command(dual_arm_control_api_t* api,
                                                 int follow,
                                                 uint8_t trajectory_mode,
                                                 uint16_t radio);

void dual_arm_reset_joint_target_cache(dual_arm_control_api_t* api);

dual_arm_ret_code_t dual_arm_get_joint(dual_arm_control_api_t* api,
                                       dual_arm_group_t group,
                                       float* out_joints, size_t out_capacity,
                                       size_t* out_count);

dual_arm_ret_code_t dual_arm_get_joint_velocity(dual_arm_control_api_t* api,
                                                dual_arm_group_t group,
                                                float* out_velocities,
                                                size_t out_capacity,
                                                size_t* out_count);

dual_arm_ret_code_t dual_arm_set_joint_max_speed(dual_arm_control_api_t* api,
                                                 dual_arm_group_t group,
                                                 const float* limits,
                                                 size_t limit_count,
                                                 float* out_limits,
                                                 size_t out_capacity,
                                                 size_t* out_count);

dual_arm_ret_code_t dual_arm_set_joint_max_acc(dual_arm_control_api_t* api,
                                               dual_arm_group_t group,
                                               const float* limits,
                                               size_t limit_count,
                                               float* out_limits,
                                               size_t out_capacity,
                                               size_t* out_count);

dual_arm_ret_code_t dual_arm_get_joint_max_limit(dual_arm_control_api_t* api,
                                                 dual_arm_group_t group,
                                                 float* out_limits,
                                                 size_t out_capacity,
                                                 size_t* out_count);

dual_arm_ret_code_t dual_arm_get_joint_min_limit(dual_arm_control_api_t* api,
                                                 dual_arm_group_t group,
                                                 float* out_limits,
                                                 size_t out_capacity,
                                                 size_t* out_count);

dual_arm_ret_code_t dual_arm_set_gripper_config(dual_arm_control_api_t* api,
                                                dual_arm_group_t group,
                                                uint16_t speed, uint16_t force,
                                                int block, int timeout_sec);

dual_arm_ret_code_t dual_arm_set_gripper_position(dual_arm_control_api_t* api,
                                                  dual_arm_group_t group,
                                                  uint16_t position, int block,
                                                  int timeout_sec);

dual_arm_ret_code_t dual_arm_get_gripper_state(dual_arm_control_api_t* api,
                                               dual_arm_group_t group,
                                               dual_arm_gripper_state_t* out_state);

#ifdef __cplusplus
}
#endif

#endif
