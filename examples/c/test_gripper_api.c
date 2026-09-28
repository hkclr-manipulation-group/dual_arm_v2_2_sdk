/**
 * @file test_gripper_api.c
 * @brief 单夹爪 API 交互测试（C API）。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>

static int print_state(dual_arm_control_api_t* api) {
    dual_arm_gripper_state_t state;
    const dual_arm_ret_code_t code =
        dual_arm_get_gripper_state(api, DUAL_ARM_GROUP_LEFT_ARM, &state);
    printf("get_gripper_state: %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) {
        return 0;
    }
    printf("  position: %u / 1000\n", state.position);
    printf("  speed:    %u / 1000\n", state.speed);
    printf("  force:    %u / 1000\n", state.force);
    return 1;
}

int main(void) {
    char config_path[kMaxConfigPathLen];
    default_config_path(config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    printf("Waiting for rt_control gripper state...\n");
    int received = 0;
    dual_arm_ret_code_t last_gripper_code = DUAL_ARM_RET_RECEIVE_FAILED;
    for (int attempt = 0; attempt < 150; ++attempt) {
        dual_arm_gripper_state_t state;
        last_gripper_code =
            dual_arm_get_gripper_state(api, DUAL_ARM_GROUP_LEFT_ARM, &state);
        if (last_gripper_code == DUAL_ARM_RET_SUCCESS) {
            received = 1;
            break;
        }
        sleep_ms(20);
    }

    if (!received) {
        float joints[kMaxJoints];
        size_t count = 0;
        if (dual_arm_get_joint(api, DUAL_ARM_GROUP_LEFT_ARM, joints, kMaxJoints,
                               &count) != DUAL_ARM_RET_SUCCESS) {
            fprintf(stderr,
                    "No rt_control state received within 3 seconds.\n");
        } else if (last_gripper_code == DUAL_ARM_RET_CONTROLLER_ERROR) {
            fprintf(stderr,
                    "rt_control state is arriving, but gripper reports error.\n");
        } else {
            fprintf(stderr,
                    "rt_control state is arriving, but no left-gripper state.\n");
        }
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!print_state(api)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Configure speed=200 (72 deg/s output shaft), force=500 (0.5 Nm).")) {
        dual_arm_control_api_destroy(api);
        return 0;
    }
    dual_arm_ret_code_t code = dual_arm_set_gripper_config(
        api, DUAL_ARM_GROUP_LEFT_ARM, 1000, 500, 1, 2);
    printf("set_gripper_config: %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter("Open the gripper to position 0.")) {
        dual_arm_control_api_destroy(api);
        return 0;
    }
    code = dual_arm_set_gripper_position(api, DUAL_ARM_GROUP_LEFT_ARM, 0, 1,
                                         10);
    printf("set_gripper_position(open): %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS || !print_state(api)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!wait_for_enter(
            "Close the gripper to position 1000; contact torque can finish early.")) {
        dual_arm_control_api_destroy(api);
        return 0;
    }
    code = dual_arm_set_gripper_position(api, DUAL_ARM_GROUP_LEFT_ARM, 1000, 1,
                                         10);
    printf("set_gripper_position(close): %s\n", dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS || !print_state(api)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Gripper test completed.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
