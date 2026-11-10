/**
 * @file go_home.c
 * @brief 双臂回零（C API）：左/右臂各 7 关节置 0，头部保持当前角度。
 *
 * 前置条件：
 *   - rt_control 已启动，且 config.yaml 可被自动找到或显式传入
 *
 * 用法（在 build/ 或 build/bin/ 目录下均可）：
 *   ./bin/go_home
 *   ./go_home /path/to/cuarm_configuration/dual_v2_2
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>

int main(int argc, char* argv[]) {
    char config_path[kMaxConfigPathLen];
    config_path_from_args(argc, argv, config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float probe[16];
    size_t probe_count = 0;
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, probe,
                              sizeof(probe) / sizeof(probe[0]), &probe_count,
                              kDefaultStateTimeoutMs)) {
        fprintf(stderr,
                "No state received within 3 s. Is rt_control running?\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_RIGHT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "Failed to enable arms\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float head[8];
    float left[8];
    size_t head_count = 0;
    size_t left_count = 0;
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head,
                              sizeof(head) / sizeof(head[0]), &head_count,
                              kDefaultStateTimeoutMs) ||
        !wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left,
                              sizeof(left) / sizeof(left[0]), &left_count,
                              kDefaultStateTimeoutMs)) {
        fprintf(stderr, "Failed to read joint state\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    float all[32];
    size_t all_count = 0;
    for (size_t i = 0; i < left_count; ++i) {
        all[all_count++] = 0.0F;
    }
    for (size_t i = 0; i < head_count; ++i) {
        all[all_count++] = head[i];
    }
    for (size_t i = 0; i < left_count; ++i) {
        all[all_count++] = 0.0F;
    }

    if (dual_arm_move_joint(api, DUAL_ARM_GROUP_ALL, all, all_count, 0, 0,
                            0, 0) != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "move_joint failed\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Both arms moving to home (arm joints zero, head unchanged)\n");
    dual_arm_control_api_destroy(api);
    return 0;
}
