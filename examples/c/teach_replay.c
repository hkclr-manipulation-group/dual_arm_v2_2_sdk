/**
 * @file teach_replay.c
 * @brief 示教点回放：循环播放关节空间轨迹，Ctrl+C 停止。
 *
 * 流程：
 *   1. 等待 rt_control 状态
 *   2. 启用双臂
 *   3. 从文件加载示教点并循环回放（低跟随 move_joint）
 *
 * 示教文件格式（逗号分隔，单位：度）：
 *   idx, left_j1..j7, [left_gripper,] right_j1..j7, [right_gripper]
 *   支持 15 列（无夹爪）或 17 列（含夹爪；夹爪列会被解析但不回放）
 *
 * 用法（在 build/ 或 build/bin/ 目录下均可）：
 *   ./bin/teach_replay
 *   ./teach_replay
 *   ./bin/teach_replay /path/to/teach_points_left_right.txt
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const float kMaxSpeedDegS = 50.0F;
static const float kPositionToleranceDeg = 0.5F;
static const int kReachTimeoutSeconds = 10;
static const float kDwellSeconds = 1.0F;

static volatile sig_atomic_t g_stop_requested = 0;

typedef struct {
    char label[64];
    float left_rad[7];
    float right_rad[7];
} teach_point_t;

static void handle_sigint(int signo) {
    (void)signo;
    g_stop_requested = 1;
}

static void deg_list_to_rad(const float* degrees, float* radians, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        radians[i] = degrees[i] * kDegToRad;
    }
}

static void print_joints_deg(const char* prefix, const float* joints_rad,
                             size_t count) {
    printf("%s: [", prefix);
    for (size_t i = 0; i < count; ++i) {
        printf("%.3f", joints_rad[i] * kRadToDeg);
        if (i + 1 < count) {
            printf(", ");
        }
    }
    printf("]\n");
}

static int split_csv_line(char* line, char** cols, int max_cols) {
    int count = 0;
    char* token = line;
    while (token != NULL && count < max_cols) {
        cols[count++] = trim_inplace(token);
        char* comma = strchr(token, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        token = comma + 1;
    }
    return count;
}

static int load_teach_points(const char* filepath, teach_point_t** out_points,
                             size_t* out_count) {
    FILE* input = fopen(filepath, "r");
    if (input == NULL) {
        fprintf(stderr, "Failed to open teach file: %s\n", filepath);
        return 0;
    }

    teach_point_t* points = NULL;
    size_t capacity = 0;
    size_t count = 0;
    char line[1024];

    while (fgets(line, (int)sizeof(line), input) != NULL) {
        trim_inplace(line);
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }

        char* cols[32];
        char line_copy[1024];
        snprintf(line_copy, sizeof(line_copy), "%s", line);
        const int col_count = split_csv_line(line_copy, cols, 32);
        if (col_count != 17 && col_count != 15) {
            printf("Skip malformed line (expected 15 or 17 columns, got %d): "
                   "%s\n",
                   col_count, line);
            continue;
        }

        if (count >= capacity) {
            capacity = capacity == 0 ? 8 : capacity * 2;
            teach_point_t* grown =
                (teach_point_t*)realloc(points, capacity * sizeof(*points));
            if (grown == NULL) {
                fprintf(stderr, "Out of memory loading teach points\n");
                free(points);
                fclose(input);
                return 0;
            }
            points = grown;
        }

        teach_point_t* point = &points[count];
        snprintf(point->label, sizeof(point->label), "%s", cols[0]);

        float left_deg[7];
        float right_deg[7];
        for (int i = 0; i < 7; ++i) {
            left_deg[i] = strtof(cols[1 + i], NULL);
        }
        const int right_start = col_count == 17 ? 9 : 8;
        for (int i = 0; i < 7; ++i) {
            right_deg[i] = strtof(cols[right_start + i], NULL);
        }
        deg_list_to_rad(left_deg, point->left_rad, 7);
        deg_list_to_rad(right_deg, point->right_rad, 7);
        ++count;
    }

    fclose(input);
    *out_points = points;
    *out_count = count;
    return 1;
}

static int wait_for_reach(dual_arm_control_api_t* api,
                          const float* left_target, size_t left_count,
                          const float* right_target, size_t right_count,
                          float tolerance_rad, int poll_interval_ms,
                          int timeout_seconds, int settle_reads) {
    int elapsed_ms = 0;
    int stable = 0;
    float last_error = 0.0F;
    int progress_ms = 0;

    while (elapsed_ms <= timeout_seconds * 1000) {
        float left_current[kMaxJoints];
        float right_current[kMaxJoints];
        size_t left_current_count = 0;
        size_t right_current_count = 0;
        const dual_arm_ret_code_t left_code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_LEFT_ARM, left_current, kMaxJoints,
            &left_current_count);
        const dual_arm_ret_code_t right_code = dual_arm_get_joint(
            api, DUAL_ARM_GROUP_RIGHT_ARM, right_current, kMaxJoints,
            &right_current_count);

        if (left_code != DUAL_ARM_RET_SUCCESS ||
            right_code != DUAL_ARM_RET_SUCCESS ||
            left_current_count != left_count ||
            right_current_count != right_count) {
            sleep_ms(poll_interval_ms);
            elapsed_ms += poll_interval_ms;
            progress_ms += poll_interval_ms;
            continue;
        }

        float max_error = max_joint_error(left_target, left_current, left_count);
        max_error = float_max(
            max_error, max_joint_error(right_target, right_current, right_count));
        last_error = max_error;

        if (max_error <= tolerance_rad) {
            if (++stable >= settle_reads) {
                printf("    reached target (max joint err %.3f deg)\n",
                       max_error * kRadToDeg);
                return 1;
            }
        } else {
            stable = 0;
        }

        if (progress_ms >= 5000) {
            printf("    moving... max joint err %.3f deg (%d s)\n",
                   max_error * kRadToDeg, elapsed_ms / 1000);
            progress_ms = 0;
        } else {
            progress_ms += poll_interval_ms;
        }

        sleep_ms(poll_interval_ms);
        elapsed_ms += poll_interval_ms;
    }

    printf("    WARNING: timed out after %d s (last max joint err %.3f deg)\n",
           timeout_seconds, last_error * kRadToDeg);
    return 0;
}

static int move_both_arms(dual_arm_control_api_t* api,
                          const float* left_target, size_t left_count,
                          const float* right_target, size_t right_count,
                          const float* head_hold, size_t head_count) {
    float all[kMaxJoints];
    size_t all_count = 0;
    concat_three_groups(left_target, left_count, head_hold, head_count,
                        right_target, right_count, all, kMaxJoints,
                        &all_count);

    const dual_arm_ret_code_t code =
        dual_arm_move_joint(api, DUAL_ARM_GROUP_ALL, all, all_count, 0, 0, 0);
    if (code != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "move_joint(ALL): %s\n",
                dual_arm_ret_code_name(code));
        return 0;
    }
    return 1;
}

static int configure_arm_speed(dual_arm_control_api_t* api,
                               dual_arm_group_t group, size_t joint_count) {
    float requested[kMaxJoints];
    float applied[kMaxJoints];
    size_t applied_count = 0;
    for (size_t i = 0; i < joint_count; ++i) {
        requested[i] = kMaxSpeedDegS * kDegToRad;
    }
    if (dual_arm_set_joint_max_speed(api, group, requested, joint_count,
                                     applied, kMaxJoints,
                                     &applied_count) != DUAL_ARM_RET_SUCCESS ||
        applied_count != joint_count) {
        fprintf(stderr, "set_joint_max_speed failed for group %s\n",
                dual_arm_group_name(group));
        return 0;
    }
    return 1;
}

int main(int argc, char* argv[]) {
    char teach_file[kMaxPathLen];
    char config_path[kMaxConfigPathLen];

    if (argc > 1) {
        snprintf(teach_file, sizeof(teach_file), "%s", argv[1]);
    } else {
        default_teach_file_path(teach_file, sizeof(teach_file));
    }
    default_config_path(config_path, sizeof(config_path));

    dual_arm_control_api_t* api = dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float left_current[kMaxJoints];
    float head_current[kMaxJoints];
    size_t left_count = 0;
    size_t head_count = 0;

    printf("Waiting for rt_control state...\n");
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, left_current,
                              kMaxJoints, &left_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr,
                "No state received within 3 seconds. Is rt_control running, "
                "and do the UDP addresses/ports match config.yaml?\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, head_current,
                              kMaxJoints, &head_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No head state received within 3 seconds.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (dual_arm_set_group(api, DUAL_ARM_GROUP_LEFT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS ||
        dual_arm_set_group(api, DUAL_ARM_GROUP_RIGHT_ARM, 1) !=
            DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "Failed to enable both arms.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Both arms enabled\n");

    if (!configure_arm_speed(api, DUAL_ARM_GROUP_LEFT_ARM, left_count) ||
        !configure_arm_speed(api, DUAL_ARM_GROUP_RIGHT_ARM, left_count)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Joint max speed set to %.0f deg/s\n", kMaxSpeedDegS);

    teach_point_t* points = NULL;
    size_t point_count = 0;
    if (!load_teach_points(teach_file, &points, &point_count)) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Loaded %zu point(s) from %s\n", point_count, teach_file);
    if (point_count == 0) {
        printf("No points to replay.\n");
        free(points);
        dual_arm_control_api_destroy(api);
        return 0;
    }

    const float tolerance_rad = kPositionToleranceDeg * kDegToRad;

    signal(SIGINT, handle_sigint);
    printf("Loop replay started. Press Ctrl+C to stop.\n");

    int cycle = 0;
    while (!g_stop_requested) {
        ++cycle;
        printf("\n========== Cycle %d ==========\n", cycle);

        for (size_t i = 0; i < point_count && !g_stop_requested; ++i) {
            const teach_point_t* point = &points[i];
            printf("\n==> Replay point #%zu (record idx %s):\n", i + 1,
                   point->label);
            print_joints_deg("    left_j", point->left_rad, 7);
            print_joints_deg("    right_j", point->right_rad, 7);

            if (!move_both_arms(api, point->left_rad, 7, point->right_rad, 7,
                                head_current, head_count)) {
                free(points);
                dual_arm_control_api_destroy(api);
                return 1;
            }
            wait_for_reach(api, point->left_rad, 7, point->right_rad, 7,
                           tolerance_rad, 200, kReachTimeoutSeconds, 3);

            if (g_stop_requested) {
                break;
            }

            printf("    dwell %d s at point\n", (int)kDwellSeconds);
            int dwell_ms = (int)(kDwellSeconds * 1000.0F);
            int dwell_elapsed = 0;
            while (dwell_elapsed < dwell_ms && !g_stop_requested) {
                sleep_ms(50);
                dwell_elapsed += 50;
            }
        }
    }

    printf("\nCtrl+C received, stopping motion...\n");
    printf("Motion stopped\n");
    free(points);
    dual_arm_control_api_destroy(api);
    return 0;
}
