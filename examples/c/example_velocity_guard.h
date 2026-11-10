#ifndef DUAL_ARM_V2_2_EXAMPLE_VELOCITY_GUARD_H
#define DUAL_ARM_V2_2_EXAMPLE_VELOCITY_GUARD_H

#include "example_common.h"

#include <pthread.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dual_arm_control_api_t* api;
    dual_arm_group_t group;
    float max_velocity_rad_s;
    volatile int stop;
    volatile int triggered;
    pthread_t thread;
    int started;
} example_velocity_guard_t;

static void* example_velocity_guard_thread(void* arg) {
    example_velocity_guard_t* guard = (example_velocity_guard_t*)arg;
    int over_limit_count[kMaxJoints];
    size_t over_limit_size = 0;

    while (!guard->stop) {
        float velocity[kMaxJoints];
        size_t count = 0;
        const dual_arm_ret_code_t code = dual_arm_get_joint_velocity(
            guard->api, guard->group, velocity, kMaxJoints, &count);
        if (code == DUAL_ARM_RET_SUCCESS) {
            if (over_limit_size != count) {
                over_limit_size = count;
                for (size_t i = 0; i < count; ++i) {
                    over_limit_count[i] = 0;
                }
            }
            for (size_t i = 0; i < count; ++i) {
                if (float_abs(velocity[i]) <= guard->max_velocity_rad_s) {
                    over_limit_count[i] = 0;
                    continue;
                }
                if (++over_limit_count[i] < 3) {
                    continue;
                }
                if (!guard->triggered) {
                    guard->triggered = 1;
                    fprintf(stderr,
                            "\nSAFETY STOP: J%zu velocity is %.2f deg/s, "
                            "exceeding limit for 3 consecutive samples.\n",
                            i + 1, velocity[i] * kRadToDeg);
                    float position[kMaxJoints];
                    size_t pos_count = 0;
                    if (dual_arm_get_joint(guard->api, guard->group, position,
                                           kMaxJoints,
                                           &pos_count) == DUAL_ARM_RET_SUCCESS) {
                        const dual_arm_ret_code_t hold = dual_arm_move_joint(
                            guard->api, guard->group, position, pos_count, 1,
                            0, 0, 0);
                        fprintf(stderr, "Current-position hold command: %s\n",
                                dual_arm_ret_code_name(hold));
                    } else {
                        fprintf(stderr,
                                "Could not read current position; hold not "
                                "sent.\n");
                    }
                }
                break;
            }
        }
        sleep_ms(10);
    }
    return NULL;
}

static inline int example_velocity_guard_start(example_velocity_guard_t* guard,
                                               dual_arm_control_api_t* api,
                                               dual_arm_group_t group,
                                               float max_velocity_rad_s) {
    guard->api = api;
    guard->group = group;
    guard->max_velocity_rad_s = max_velocity_rad_s;
    guard->stop = 0;
    guard->triggered = 0;
    guard->started = 0;
    if (pthread_create(&guard->thread, NULL, example_velocity_guard_thread,
                       guard) != 0) {
        return 0;
    }
    guard->started = 1;
    return 1;
}

static inline void example_velocity_guard_stop(example_velocity_guard_t* guard) {
    if (guard == NULL || !guard->started) {
        return;
    }
    guard->stop = 1;
    pthread_join(guard->thread, NULL);
    guard->started = 0;
}

static inline int example_velocity_guard_triggered(
    const example_velocity_guard_t* guard) {
    return guard != NULL && guard->triggered;
}

#ifdef __cplusplus
}
#endif

#endif
