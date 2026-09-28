/**
 * @file test_dual_gripper_api.c
 * @brief 双臂夹爪 API 交互测试（C API）。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>
#include <string.h>

static const uint16_t kGripperOpen = 0;
static const uint16_t kGripperClosed = 1000;
static const uint16_t kGripperSpeed = 400;
static const uint16_t kGripperForce = 500;
static const int kGripperTimeoutSeconds = 10;

static int print_state(dual_arm_control_api_t* api, dual_arm_group_t group) {
    dual_arm_gripper_state_t state;
    const dual_arm_ret_code_t code = dual_arm_get_gripper_state(api, group, &state);
    printf("%s get_gripper_state: %s\n", dual_arm_group_name(group),
           dual_arm_ret_code_name(code));
    if (code != DUAL_ARM_RET_SUCCESS) {
        return 0;
    }
    printf("  position: %u / 1000\n", state.position);
    printf("  speed:    %u / 1000\n", state.speed);
    printf("  force:    %u / 1000\n", state.force);
    return 1;
}

static int wait_for_gripper_state(dual_arm_control_api_t* api,
                                  dual_arm_group_t group, int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        dual_arm_gripper_state_t state;
        const dual_arm_ret_code_t code =
            dual_arm_get_gripper_state(api, group, &state);
        if (code == DUAL_ARM_RET_SUCCESS) {
            printf("%s gripper online: position=%u, speed=%u, force=%u\n",
                   dual_arm_group_name(group), state.position, state.speed,
                   state.force);
            return 1;
        }
        sleep_ms(20);
        elapsed += 20;
    }
    return 0;
}

static int configure_gripper(dual_arm_control_api_t* api,
                             dual_arm_group_t group) {
    const dual_arm_ret_code_t code = dual_arm_set_gripper_config(
        api, group, kGripperSpeed, kGripperForce, 1, 2);
    printf("%s set_gripper_config: %s\n", dual_arm_group_name(group),
           dual_arm_ret_code_name(code));
    return code == DUAL_ARM_RET_SUCCESS;
}

static int move_gripper(dual_arm_control_api_t* api, dual_arm_group_t group,
                        uint16_t position, const char* label) {
    const dual_arm_ret_code_t code = dual_arm_set_gripper_position(
        api, group, position, 1, kGripperTimeoutSeconds);
    printf("%s %s: %s\n", dual_arm_group_name(group), label,
           dual_arm_ret_code_name(code));
    return code == DUAL_ARM_RET_SUCCESS && print_state(api, group);
}

static int test_one_gripper(dual_arm_control_api_t* api,
                            dual_arm_group_t group, const char* stage_prefix) {
    char prompt[256];
    if (!configure_gripper(api, group)) {
        return 0;
    }

    snprintf(prompt, sizeof(prompt), "%s: open %s gripper.", stage_prefix,
             dual_arm_group_name(group));
    if (!wait_for_enter(prompt)) {
        return 0;
    }
    if (!move_gripper(api, group, kGripperOpen, "set_gripper_position(open)")) {
        return 0;
    }

    snprintf(prompt, sizeof(prompt), "%s: close %s gripper.", stage_prefix,
             dual_arm_group_name(group));
    if (!wait_for_enter(prompt)) {
        return 0;
    }
    if (!move_gripper(api, group, kGripperClosed,
                      "set_gripper_position(close)")) {
        return 0;
    }
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

    printf("Dual gripper test (LEFT + RIGHT)\n");

    const int left_online =
        wait_for_gripper_state(api, DUAL_ARM_GROUP_LEFT_ARM, 3000);
    const int right_online =
        wait_for_gripper_state(api, DUAL_ARM_GROUP_RIGHT_ARM, 3000);

    if (!left_online && !right_online) {
        fprintf(stderr, "No gripper state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!left_online) {
        printf("LEFT gripper unavailable; continuing with RIGHT only.\n");
    }
    if (!right_online) {
        printf("RIGHT gripper unavailable; continuing with LEFT only.\n");
    }

    if (left_online) {
        if (!wait_for_enter("Stage 1: test LEFT gripper open/close.")) {
            dual_arm_control_api_destroy(api);
            return 0;
        }
        if (!test_one_gripper(api, DUAL_ARM_GROUP_LEFT_ARM, "Stage 1")) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
    }

    if (right_online) {
        if (!wait_for_enter("Stage 2: test RIGHT gripper open/close.")) {
            dual_arm_control_api_destroy(api);
            return 0;
        }
        if (!test_one_gripper(api, DUAL_ARM_GROUP_RIGHT_ARM, "Stage 2")) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
    }

    if (left_online && right_online) {
        if (!wait_for_enter(
                "Stage 3: open both grippers, then close both grippers.")) {
            dual_arm_control_api_destroy(api);
            return 0;
        }

        dual_arm_ret_code_t left_code = dual_arm_set_gripper_position(
            api, DUAL_ARM_GROUP_LEFT_ARM, kGripperOpen, 1,
            kGripperTimeoutSeconds);
        dual_arm_ret_code_t right_code = dual_arm_set_gripper_position(
            api, DUAL_ARM_GROUP_RIGHT_ARM, kGripperOpen, 1,
            kGripperTimeoutSeconds);
        printf("Both open: LEFT=%s, RIGHT=%s\n",
               dual_arm_ret_code_name(left_code),
               dual_arm_ret_code_name(right_code));
        if (left_code != DUAL_ARM_RET_SUCCESS ||
            right_code != DUAL_ARM_RET_SUCCESS ||
            !print_state(api, DUAL_ARM_GROUP_LEFT_ARM) ||
            !print_state(api, DUAL_ARM_GROUP_RIGHT_ARM)) {
            dual_arm_control_api_destroy(api);
            return 1;
        }

        left_code = dual_arm_set_gripper_position(
            api, DUAL_ARM_GROUP_LEFT_ARM, kGripperClosed, 1,
            kGripperTimeoutSeconds);
        right_code = dual_arm_set_gripper_position(
            api, DUAL_ARM_GROUP_RIGHT_ARM, kGripperClosed, 1,
            kGripperTimeoutSeconds);
        printf("Both close: LEFT=%s, RIGHT=%s\n",
               dual_arm_ret_code_name(left_code),
               dual_arm_ret_code_name(right_code));
        if (left_code != DUAL_ARM_RET_SUCCESS ||
            right_code != DUAL_ARM_RET_SUCCESS ||
            !print_state(api, DUAL_ARM_GROUP_LEFT_ARM) ||
            !print_state(api, DUAL_ARM_GROUP_RIGHT_ARM)) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
    }

    printf("Dual gripper test completed.\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
