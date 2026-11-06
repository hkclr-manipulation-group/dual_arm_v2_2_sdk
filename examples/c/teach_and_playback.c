/**
 * @file teach_and_playback.c
 * @brief 重力补偿示教采点（C API）：双臂拖动 + 键盘记录关节角到 txt。
 *
 * 文件格式与 teach_replay.c 相同（17 列，单位：度）。
 * 回放请使用 teach_replay。
 */

#include "dual_arm_control_api.h"
#include "example_common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static void append_joint_line(dual_arm_control_api_t* api,
                              const char* filepath, int index) {
    float left_rad[7];
    float right_rad[7];
    size_t left_count = 0;
    size_t right_count = 0;

    if (dual_arm_get_joint(api, DUAL_ARM_GROUP_LEFT_ARM, left_rad,
                           sizeof(left_rad) / sizeof(left_rad[0]),
                           &left_count) != DUAL_ARM_RET_SUCCESS ||
        dual_arm_get_joint(api, DUAL_ARM_GROUP_RIGHT_ARM, right_rad,
                           sizeof(right_rad) / sizeof(right_rad[0]),
                           &right_count) != DUAL_ARM_RET_SUCCESS ||
        left_count != 7 || right_count != 7) {
        printf("  -> Failed to read joint feedback, point not saved\n");
        return;
    }

    ensure_teach_file_parent_dirs(filepath);
    FILE* out = fopen(filepath, "a");
    if (out == NULL) {
        return;
    }

    int write_header = 1;
    {
        FILE* in = fopen(filepath, "r");
        if (in != NULL) {
            int ch = fgetc(in);
            write_header = (ch == EOF);
            fclose(in);
        }
    }
    if (write_header) {
        fprintf(out, "# Teach points (unit: deg)\n");
        fprintf(out,
                "# Columns: idx, left_j1..left_j7, left_gripper, "
                "right_j1..right_j7, right_gripper\n");
    }

    fprintf(out, "%d", index);
    for (int i = 0; i < 7; ++i) {
        fprintf(out, ", %.3f", left_rad[i] * kRadToDeg);
    }
    fprintf(out, ", 0.000");
    for (int i = 0; i < 7; ++i) {
        fprintf(out, ", %.3f", right_rad[i] * kRadToDeg);
    }
    fprintf(out, ", 0.000\n");
    fclose(out);
    printf("  -> Saved point #%d\n", index);
}

static int record_teach_points(dual_arm_control_api_t* api,
                               const char* filepath) {
    const float debounce_s = 0.4F;
    int count = 0;
    struct timespec debounce_start = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &debounce_start);
    debounce_start.tv_sec -= 2;

    struct termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);
    struct termios newt = oldt;
    newt.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (1) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 100000};
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) <= 0) {
            continue;
        }

        char key = 0;
        if (read(STDIN_FILENO, &key, 1) != 1) {
            continue;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        const float elapsed =
            (float)(now.tv_sec - debounce_start.tv_sec) +
            (float)(now.tv_nsec - debounce_start.tv_nsec) / 1e9F;

        if ((key == 's' || key == ' ') && elapsed >= debounce_s) {
            ++count;
            append_joint_line(api, filepath, count);
            debounce_start = now;
        } else if (key == 'q' || key == '\r' || key == '\n') {
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return count;
}

int main(int argc, char* argv[]) {
    char config_path[kMaxConfigPathLen];
    char record_file[kMaxPathLen];

    if (argc > 1) {
        snprintf(record_file, sizeof(record_file), "%s", argv[1]);
    } else {
        default_teach_file_path(record_file, sizeof(record_file));
    }
    default_config_path(config_path, sizeof(config_path));
    ensure_teach_file_parent_dirs(record_file);

    dual_arm_control_api_t* api =
        dual_arm_control_api_create(config_path);
    if (api == NULL) {
        fprintf(stderr, "create failed: %s\n",
                dual_arm_control_api_last_error(api));
        return 1;
    }

    float joints[kMaxJoints];
    size_t joint_count = 0;

    printf("Waiting for rt_control state...\n");
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_LEFT_ARM, joints,
                              sizeof(joints) / sizeof(joints[0]),
                              &joint_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No left arm state within timeout.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }
    if (!wait_for_joint_state(api, DUAL_ARM_GROUP_HEAD, joints,
                              sizeof(joints) / sizeof(joints[0]),
                              &joint_count, kDefaultStateTimeoutMs)) {
        fprintf(stderr, "No head state within timeout.\n");
        dual_arm_control_api_destroy(api);
        return 1;
    }

    if (!check_result("set_group LEFT", dual_arm_set_group(
                                           api, DUAL_ARM_GROUP_LEFT_ARM, 1)) ||
        !check_result("set_group RIGHT", dual_arm_set_group(
                                            api, DUAL_ARM_GROUP_RIGHT_ARM, 1))) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Both arms enabled (all joints)\n");

    printf("Entering gravity-compensated teach mode...\n");
    if (!check_result("start_teach_mode",
                      dual_arm_start_teach_mode(api, 1))) {
        dual_arm_control_api_destroy(api);
        return 1;
    }
    printf("Teach mode active. Move both arms by hand.\n");
    printf("Press 's' or SPACE to record, 'q' or ENTER to stop.\n");

    const int num_points = record_teach_points(api, record_file);
    printf("Recorded %d point(s) -> %s\n", num_points, record_file);
    printf("Hand off both arms now.\n");
    sleep_ms(500);

    printf("Stopping teach mode...\n");
    if (!check_result("stop_teach_mode", dual_arm_stop_teach_mode(api))) {
        dual_arm_control_api_destroy(api);
        return 1;
    }

    printf("Done. Replay with: teach_replay %s\n", record_file);
    dual_arm_control_api_destroy(api);
    return 0;
}
