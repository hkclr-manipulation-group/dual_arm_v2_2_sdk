/**
 * @file test_get_joint_all.c
 * @brief 测试 get_joint(ALL)：读取并校验左臂 + 头部 + 右臂拼接结果。
 *
 * Group::ALL 关节顺序：LEFT_ARM (J1..J7) + HEAD (J1..J2) + RIGHT_ARM (J1..J7)
 *
 * 前置条件：rt_control 已启动。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_get_joint_all
 *   ./test_get_joint_all --watch          # 每 500 ms 刷新
 *   ./test_get_joint_all --watch 10       # 每 10 ms 刷新（仅显示 ALL，不做逐次一致性校验）
 *   ./test_get_joint_all --strict         # 静止时严格校验（容差 0.001 deg）
 *   ./test_get_joint_all /path/to/cuarm_configuration/dual_v2_2
 *
 * 注意：每次 get_joint() 独立取 UDP 状态快照，连续多次调用并非同一时刻。
 *       高频 --watch 下请依赖 get_joint(ALL) 的单次结果；一致性校验仅在非 watch
 *       模式或 --strict 静止测试时使用。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    size_t left;
    size_t head;
    size_t right;
} segment_layout_t;

typedef struct {
    char config_path[kMaxConfigPathLen];
    int watch;
    int strict;
    int watch_interval_ms;
} options_t;

static volatile sig_atomic_t g_stop_requested = 0;

static void handle_sigint(int signo) {
    (void)signo;
    g_stop_requested = 1;
}

static void print_usage(const char* prog) {
    fprintf(stderr,
            "Usage:\n"
            "  %s [config_prefix] [--watch [interval_ms]] [--strict]\n\n"
            "Examples:\n"
            "  %s\n"
            "  %s --watch\n"
            "  %s --watch 10\n"
            "  %s --strict\n",
            prog, prog, prog, prog, prog);
}

static int parse_options(int argc, char* argv[], options_t* opts) {
    opts->watch = 0;
    opts->strict = 0;
    opts->watch_interval_ms = 500;
    config_path_from_args(argc, argv, opts->config_path,
                          sizeof(opts->config_path));

    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(arg, "--watch") == 0) {
            opts->watch = 1;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opts->watch_interval_ms = atoi(argv[i + 1]);
                if (opts->watch_interval_ms <= 0) {
                    opts->watch_interval_ms = 500;
                }
                ++i;
            }
            continue;
        }
        if (strcmp(arg, "--strict") == 0) {
            opts->strict = 1;
            continue;
        }
        if (strncmp(arg, "--", 2) == 0) {
            fprintf(stderr, "Unknown option: %s\n", arg);
            print_usage(argv[0]);
            return 0;
        }
    }
    return 1;
}

static void print_joints_deg(const char* label, const float* joints_rad,
                             size_t count) {
    printf("%s (%zu joints, deg): [", label, count);
    for (size_t i = 0; i < count; ++i) {
        printf("%.3f", joints_rad[i] * kRadToDeg);
        if (i + 1 < count) {
            printf(", ");
        }
    }
    printf("]\n");
}

static void print_segment(const char* segment_label, size_t offset,
                          size_t count, const float* all_rad,
                          size_t all_count) {
    printf("  %s: ", segment_label);
    if (count == 0 || offset >= all_count) {
        printf("(empty)\n");
        return;
    }
    const size_t end = offset + count;
    const size_t clamped_end = end > all_count ? all_count : end;
    for (size_t i = offset; i < clamped_end; ++i) {
        printf("J%zu=%.3f deg", i - offset + 1, all_rad[i] * kRadToDeg);
        if (i + 1 < clamped_end) {
            printf(", ");
        }
    }
    printf("\n");
}

static float max_abs_diff(const float* a, const float* b, size_t n) {
    float max_diff = 0.0F;
    for (size_t i = 0; i < n; ++i) {
        max_diff = float_max(max_diff, float_abs(a[i] - b[i]));
    }
    return max_diff;
}

static float max_abs_value(const float* values, size_t count) {
    float max_value = 0.0F;
    for (size_t i = 0; i < count; ++i) {
        max_value = float_max(max_value, float_abs(values[i]));
    }
    return max_value;
}

static double monotonic_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int probe_segment_layout(dual_arm_control_api_t* api,
                                segment_layout_t* layout) {
    float left[kMaxJoints];
    float head[kMaxJoints];
    float right[kMaxJoints];
    size_t left_count = 0;
    size_t head_count = 0;
    size_t right_count = 0;

    if (dual_arm_get_joint(api, DUAL_ARM_GROUP_LEFT_ARM, left, kMaxJoints,
                           &left_count) != DUAL_ARM_RET_SUCCESS ||
        dual_arm_get_joint(api, DUAL_ARM_GROUP_HEAD, head, kMaxJoints,
                           &head_count) != DUAL_ARM_RET_SUCCESS ||
        dual_arm_get_joint(api, DUAL_ARM_GROUP_RIGHT_ARM, right, kMaxJoints,
                           &right_count) != DUAL_ARM_RET_SUCCESS) {
        return 0;
    }

    layout->left = left_count;
    layout->head = head_count;
    layout->right = right_count;
    return 1;
}

static int read_watch(dual_arm_control_api_t* api,
                      const segment_layout_t* layout) {
    float all_joints[kMaxJoints];
    float all_vel[kMaxJoints];
    size_t all_count = 0;
    size_t vel_count = 0;

    const dual_arm_ret_code_t all_code =
        dual_arm_get_joint(api, DUAL_ARM_GROUP_ALL, all_joints, kMaxJoints,
                           &all_count);
    const dual_arm_ret_code_t vel_code = dual_arm_get_joint_velocity(
        api, DUAL_ARM_GROUP_ALL, all_vel, kMaxJoints, &vel_count);

    const size_t expected_total = layout->left + layout->head + layout->right;

    printf("\n--- get_joint(ALL) ---\n");
    printf("  status: %s  size=%zu\n", dual_arm_ret_code_name(all_code),
           all_count);
    if (all_code != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "get_joint(ALL) failed.\n");
        return 0;
    }
    if (all_count != expected_total) {
        fprintf(stderr, "Unexpected ALL size: got %zu, expected %zu\n",
                all_count, expected_total);
        return 0;
    }

    print_joints_deg("ALL", all_joints, all_count);
    if (vel_code == DUAL_ARM_RET_SUCCESS && vel_count == all_count) {
        print_joints_deg("ALL velocity", all_vel, all_count);
    }

    printf("\nALL segments (deg):\n");
    print_segment("LEFT_ARM", 0, layout->left, all_joints, all_count);
    print_segment("HEAD", layout->left, layout->head, all_joints, all_count);
    print_segment("RIGHT_ARM", layout->left + layout->head, layout->right,
                  all_joints, all_count);
    return 1;
}

static int read_once(dual_arm_control_api_t* api, int strict) {
    const double t_start = monotonic_seconds();

    float left_joints[kMaxJoints];
    float head_joints[kMaxJoints];
    float right_joints[kMaxJoints];
    float all_joints[kMaxJoints];
    size_t left_count = 0;
    size_t head_count = 0;
    size_t right_count = 0;
    size_t all_count = 0;

    const dual_arm_ret_code_t left_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_LEFT_ARM, left_joints, kMaxJoints, &left_count);
    const dual_arm_ret_code_t head_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_HEAD, head_joints, kMaxJoints, &head_count);
    const dual_arm_ret_code_t right_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_RIGHT_ARM, right_joints, kMaxJoints,
        &right_count);
    const dual_arm_ret_code_t all_code = dual_arm_get_joint(
        api, DUAL_ARM_GROUP_ALL, all_joints, kMaxJoints, &all_count);

    const double t_end = monotonic_seconds();
    const float elapsed_s = (float)(t_end - t_start);

    float all_vel[kMaxJoints];
    size_t vel_count = 0;
    const dual_arm_ret_code_t vel_code = dual_arm_get_joint_velocity(
        api, DUAL_ARM_GROUP_ALL, all_vel, kMaxJoints, &vel_count);

    printf("\n--- get_joint snapshot ---\n");
    printf("  get_joint(ALL):       %s  size=%zu\n",
           dual_arm_ret_code_name(all_code), all_count);
    printf("  get_joint(LEFT_ARM):  %s  size=%zu\n",
           dual_arm_ret_code_name(left_code), left_count);
    printf("  get_joint(HEAD):      %s  size=%zu\n",
           dual_arm_ret_code_name(head_code), head_count);
    printf("  get_joint(RIGHT_ARM): %s  size=%zu\n",
           dual_arm_ret_code_name(right_code), right_count);
    printf("  read span: %.3f ms (each call is an independent UDP snapshot)\n",
           elapsed_s * 1000.0F);

    if (all_code != DUAL_ARM_RET_SUCCESS || left_code != DUAL_ARM_RET_SUCCESS ||
        head_code != DUAL_ARM_RET_SUCCESS ||
        right_code != DUAL_ARM_RET_SUCCESS) {
        fprintf(stderr, "One or more get_joint calls failed.\n");
        return 0;
    }

    float expected[kMaxJoints];
    size_t expected_count = 0;
    concat_three_groups(left_joints, left_count, head_joints, head_count,
                        right_joints, right_count, expected, kMaxJoints,
                        &expected_count);

    printf("\nExpected ALL layout: LEFT(%zu) + HEAD(%zu) + RIGHT(%zu) = %zu "
           "joints\n",
           left_count, head_count, right_count, expected_count);

    print_joints_deg("ALL", all_joints, all_count);
    print_joints_deg("LEFT_ARM", left_joints, left_count);
    print_joints_deg("HEAD", head_joints, head_count);
    print_joints_deg("RIGHT_ARM", right_joints, right_count);

    if (all_count != expected_count) {
        fprintf(stderr, "FAIL: ALL size mismatch. got %zu, expected %zu\n",
                all_count, expected_count);
        return 0;
    }

    const float diff_rad = max_abs_diff(all_joints, expected, expected_count);
    const float diff_deg = diff_rad * kRadToDeg;

    float max_vel_rad_s = 0.0F;
    if (vel_code == DUAL_ARM_RET_SUCCESS) {
        max_vel_rad_s = max_abs_value(all_vel, vel_count);
    }

    const float kStrictToleranceDeg = 1e-3F;
    const float kDefaultFloorToleranceDeg = 0.05F;
    const float motion_tolerance_deg =
        max_vel_rad_s * kRadToDeg * elapsed_s * 2.0F;
    const float tolerance_deg =
        strict ? kStrictToleranceDeg
               : float_max(kDefaultFloorToleranceDeg, motion_tolerance_deg);

    printf("Consistency check: max |ALL - concat(L,H,R)| = %.6f rad (%.6f "
           "deg), tolerance %.6f deg\n",
           diff_rad, diff_deg, tolerance_deg);

    if (diff_deg > tolerance_deg) {
        fprintf(stderr,
                "FAIL: ALL joints differ from concatenated groups "
                "(likely UDP snapshot drift across sequential calls).\n");
        return 0;
    }

    printf("PASS: get_joint(ALL) matches LEFT + HEAD + RIGHT.\n");

    printf("\nALL segments (deg):\n");
    print_segment("LEFT_ARM", 0, left_count, all_joints, all_count);
    print_segment("HEAD", left_count, head_count, all_joints, all_count);
    print_segment("RIGHT_ARM", left_count + head_count, right_count,
                  all_joints, all_count);

    if (vel_code == DUAL_ARM_RET_SUCCESS && vel_count == all_count) {
        print_joints_deg("ALL velocity", all_vel, all_count);
    } else {
        printf("get_joint_velocity(ALL): %s  size=%zu\n",
               dual_arm_ret_code_name(vel_code), vel_count);
    }

    return 1;
}

int main(int argc, char* argv[]) {
    options_t opts;
    if (!parse_options(argc, argv, &opts)) {
        return argc > 1 ? 1 : 0;
    }

    dual_arm_control_api_t* api =
        dual_arm_control_api_create(opts.config_path);
    if (api == NULL) {
        fprintf(stderr, "Failed to create API: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    printf("Config: %s\n", opts.config_path);
    printf("Testing %s via get_joint()\n",
           dual_arm_group_name(DUAL_ARM_GROUP_ALL));

    float probe[kMaxJoints];
    size_t probe_count = 0;
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, probe, kMaxJoints,
                              &probe_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr,
                "No state received within 3 s. Is rt_control running?\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    segment_layout_t layout;
    if (!probe_segment_layout(api, &layout)) {
        fprintf(stderr, "Failed to probe segment layout.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!opts.watch) {
        const int ok = read_once(api, opts.strict);
        dual_arm_control_api_destroy(api);
        return ok ? 0 : 1;
    }

    signal(SIGINT, handle_sigint);
    printf("Watch mode: refresh every %d ms, only get_joint(ALL) per tick "
           "(Ctrl+C to stop)\n",
           opts.watch_interval_ms);
    printf("Layout: LEFT(%zu) + HEAD(%zu) + RIGHT(%zu)\n", layout.left,
           layout.head, layout.right);

    int iteration = 0;
    while (!g_stop_requested) {
        printf("\n========== iteration %d ==========\n", ++iteration);
        if (!read_watch(api, &layout)) {
            dual_arm_control_api_destroy(api);
            return 1;
        }
        sleep_ms(opts.watch_interval_ms);
    }

    dual_arm_control_api_destroy(api);
    return 0;
}
