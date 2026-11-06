#ifndef DUAL_ARM_V2_2_EXAMPLE_COMMON_C_H
#define DUAL_ARM_V2_2_EXAMPLE_COMMON_C_H

#include "dual_arm_control_api.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { kDefaultStateTimeoutMs = 3000 };
enum { kMaxConfigPathLen = PATH_MAX };
enum { kMaxPathLen = PATH_MAX };
enum { kMaxJoints = 32 };

static const float kPi = 3.14159265358979323846F;
static const float kDegToRad = 0.017453292519943295769F;
static const float kRadToDeg = 57.29577951308232F;

static inline int executable_directory(char* out, size_t out_size) {
    if (out == NULL || out_size == 0) {
        return 0;
    }
#if defined(__linux__)
    const ssize_t len = readlink("/proc/self/exe", out, out_size - 1);
    if (len <= 0) {
        return 0;
    }
    out[len] = '\0';
    char* slash = strrchr(out, '/');
    if (slash != NULL) {
        *slash = '\0';
    }
    return 1;
#else
    if (getcwd(out, out_size) == NULL) {
        return 0;
    }
    return 1;
#endif
}

static inline int path_exists(const char* path) {
    return access(path, F_OK) == 0;
}

static inline int join_path(char* out, size_t out_size, const char* base,
                            const char* suffix) {
    if (out == NULL || base == NULL || suffix == NULL) {
        return 0;
    }
    const int written = snprintf(out, out_size, "%s/%s", base, suffix);
    return written > 0 && (size_t)written < out_size;
}

static inline void default_config_path(char* out, size_t out_size) {
    const char* env = getenv("DUAL_ARM_CONFIG_PREFIX");
    if (env != NULL && env[0] != '\0') {
        snprintf(out, out_size, "%s", env);
        return;
    }

    char exe_dir[kMaxConfigPathLen];
    if (!executable_directory(exe_dir, sizeof(exe_dir))) {
        snprintf(out, out_size, "../../../cuarm_configuration/dual_v2_2");
        return;
    }

    const char* candidates[] = {
        "../../../cuarm_configuration/dual_v2_2",
        "../../cuarm_configuration/dual_v2_2",
    };
    char candidate[kMaxConfigPathLen];
    char config_file[kMaxConfigPathLen];

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        if (!join_path(candidate, sizeof(candidate), exe_dir, candidates[i])) {
            continue;
        }
        if (!join_path(config_file, sizeof(config_file), candidate,
                       "config.yaml")) {
            continue;
        }
        if (path_exists(config_file)) {
            snprintf(out, out_size, "%s", candidate);
            return;
        }
    }

    join_path(out, out_size, exe_dir, candidates[0]);
}

static inline void config_path_from_args(int argc, char* argv[], char* out,
                                         size_t out_size) {
    if (argc > 1 && argv[1] != NULL && argv[1][0] != '\0' &&
        argv[1][0] != '-') {
        char config_file[kMaxConfigPathLen];
        if (join_path(config_file, sizeof(config_file), argv[1], "config.yaml") &&
            path_exists(config_file)) {
            snprintf(out, out_size, "%s", argv[1]);
            return;
        }
    }
    default_config_path(out, out_size);
}

static inline int ensure_teach_file_parent_dirs(const char* filepath) {
    if (filepath == NULL || filepath[0] == '\0') {
        return 0;
    }
    char path[kMaxPathLen];
    if (snprintf(path, sizeof(path), "%s", filepath) <= 0) {
        return 0;
    }
    char* last_slash = strrchr(path, '/');
    if (last_slash == NULL) {
        return 1;
    }
    *last_slash = '\0';
    if (path[0] == '\0') {
        return 1;
    }
    for (char* p = path + 1; *p != '\0'; ++p) {
        if (*p != '/') {
            continue;
        }
        *p = '\0';
        if (path[0] != '\0') {
            (void)mkdir(path, 0755);
        }
        *p = '/';
    }
    (void)mkdir(path, 0755);
    return 1;
}

static inline void default_teach_file_path(char* out, size_t out_size) {
    char exe_dir[kMaxPathLen];
    if (!executable_directory(exe_dir, sizeof(exe_dir))) {
        snprintf(out, out_size,
                 "../../../examples/data/teach_points.txt");
        ensure_teach_file_parent_dirs(out);
        return;
    }

    const char* candidates[] = {
        "../../../examples/data/teach_points.txt",
        "../../examples/data/teach_points.txt",
    };
    char candidate[kMaxPathLen];
    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        if (join_path(candidate, sizeof(candidate), exe_dir, candidates[i]) &&
            path_exists(candidate)) {
            snprintf(out, out_size, "%s", candidate);
            ensure_teach_file_parent_dirs(out);
            return;
        }
    }
    join_path(out, out_size, exe_dir, candidates[0]);
    ensure_teach_file_parent_dirs(out);
}

static inline int wait_for_joint_state(dual_arm_control_api_t* api,
                                       dual_arm_group_t group, float* joints,
                                       size_t capacity, size_t* out_count,
                                       int timeout_ms) {
    const int step_ms = 20;
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        const dual_arm_ret_code_t code =
            dual_arm_get_joint(api, group, joints, capacity, out_count);
        if (code == DUAL_ARM_RET_SUCCESS && out_count != NULL &&
            *out_count > 0) {
            return 1;
        }
        usleep((useconds_t)step_ms * 1000U);
        elapsed += step_ms;
    }
    return 0;
}

static inline int wait_for_enter(const char* message) {
    if (message != NULL) {
        printf("\n%s\nPress Enter to continue, or Ctrl+D to abort...", message);
        fflush(stdout);
    }
    char input[8];
    if (fgets(input, (int)sizeof(input), stdin) == NULL) {
        printf("\nTest aborted by user.\n");
        return 0;
    }
    return 1;
}

static inline int check_result(const char* operation, dual_arm_ret_code_t code) {
    printf("%s: %s (%d)\n", operation, dual_arm_ret_code_name(code),
           (int)code);
    return code == DUAL_ARM_RET_SUCCESS;
}

static inline void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static inline float float_max(float a, float b) {
    return a > b ? a : b;
}

static inline float float_abs(float v) {
    return v < 0.0F ? -v : v;
}

static inline float max_joint_error(const float* target, const float* actual,
                                    size_t count) {
    float max_error = 0.0F;
    for (size_t i = 0; i < count; ++i) {
        max_error = float_max(max_error, float_abs(actual[i] - target[i]));
    }
    return max_error;
}

static inline void concat_three_groups(const float* left, size_t left_count,
                                       const float* head, size_t head_count,
                                       const float* right, size_t right_count,
                                       float* out, size_t out_capacity,
                                       size_t* out_count) {
    size_t n = 0;
    for (size_t i = 0; i < left_count && n < out_capacity; ++i) {
        out[n++] = left[i];
    }
    for (size_t i = 0; i < head_count && n < out_capacity; ++i) {
        out[n++] = head[i];
    }
    for (size_t i = 0; i < right_count && n < out_capacity; ++i) {
        out[n++] = right[i];
    }
    if (out_count != NULL) {
        *out_count = n;
    }
}

static inline char* trim_inplace(char* value) {
    if (value == NULL) {
        return value;
    }
    char* start = value;
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
        ++start;
    }
    if (start != value) {
        memmove(value, start, strlen(start) + 1);
    }
    size_t len = strlen(value);
    while (len > 0 &&
           (value[len - 1] == ' ' || value[len - 1] == '\t' ||
            value[len - 1] == '\r' || value[len - 1] == '\n')) {
        value[--len] = '\0';
    }
    return value;
}

#ifdef __cplusplus
}
#endif

#endif
