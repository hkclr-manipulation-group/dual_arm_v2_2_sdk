#include "dual_arm_control_api.h"

#include "control_api.h"

#include <cstring>
#include <string>
#include <vector>

namespace {

thread_local char g_last_error[512] = {};

void set_last_error(const char* message) {
    if (message == nullptr) {
        g_last_error[0] = '\0';
        return;
    }
    std::strncpy(g_last_error, message, sizeof(g_last_error) - 1);
    g_last_error[sizeof(g_last_error) - 1] = '\0';
}

void set_last_error(const std::exception& ex) {
    set_last_error(ex.what());
}

dual_arm_v2_2_sdk::Group to_cpp_group(dual_arm_group_t group) {
    return static_cast<dual_arm_v2_2_sdk::Group>(group);
}

dual_arm_ret_code_t to_c_ret(dual_arm_v2_2_sdk::RetCode code) {
    return static_cast<dual_arm_ret_code_t>(code);
}

dual_arm_state_t to_c_state(dual_arm_v2_2_sdk::State state) {
    return static_cast<dual_arm_state_t>(state);
}

bool copy_float_vector(const std::vector<float>& src, float* out, size_t capacity,
                       size_t* out_count) {
    if (out_count != nullptr) {
        *out_count = src.size();
    }
    if (src.empty()) {
        return true;
    }
    if (out == nullptr || capacity < src.size()) {
        set_last_error("Output buffer too small for float vector");
        return false;
    }
    std::memcpy(out, src.data(), src.size() * sizeof(float));
    return true;
}

}  // namespace

struct dual_arm_control_api {
    dual_arm_v2_2_sdk::ControlApi* cpp = nullptr;
};

extern "C" dual_arm_control_api_t* dual_arm_control_api_create(
    const char* config_prefix_path) {
    if (config_prefix_path == nullptr) {
        set_last_error("config_prefix_path is NULL");
        return nullptr;
    }
    try {
        auto* handle = new dual_arm_control_api();
        handle->cpp = new dual_arm_v2_2_sdk::ControlApi(config_prefix_path);
        set_last_error(nullptr);
        return handle;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return nullptr;
    } catch (...) {
        set_last_error("Unknown exception in dual_arm_control_api_create");
        return nullptr;
    }
}

extern "C" void dual_arm_control_api_destroy(dual_arm_control_api_t* api) {
    if (api == nullptr) {
        return;
    }
    delete api->cpp;
    delete api;
}

extern "C" const char* dual_arm_control_api_last_error(
    const dual_arm_control_api_t* /*api*/) {
    return g_last_error;
}

extern "C" const char* dual_arm_ret_code_name(dual_arm_ret_code_t code) {
    switch (code) {
        case DUAL_ARM_RET_SUCCESS:
            return "SUCCESS";
        case DUAL_ARM_RET_CONTROLLER_ERROR:
            return "CONTROLLER_ERROR";
        case DUAL_ARM_RET_SEND_FAILED:
            return "SEND_FAILED";
        case DUAL_ARM_RET_RECEIVE_FAILED:
            return "RECEIVE_FAILED";
        case DUAL_ARM_RET_PARSE_FAILED:
            return "PARSE_FAILED";
        case DUAL_ARM_RET_TIMEOUT:
            return "TIMEOUT";
    }
    return "UNKNOWN";
}

extern "C" const char* dual_arm_group_name(dual_arm_group_t group) {
    switch (group) {
        case DUAL_ARM_GROUP_ALL:
            return "ALL";
        case DUAL_ARM_GROUP_LEFT_ARM:
            return "LEFT_ARM";
        case DUAL_ARM_GROUP_RIGHT_ARM:
            return "RIGHT_ARM";
        case DUAL_ARM_GROUP_HEAD:
            return "HEAD";
        case DUAL_ARM_GROUP_ELEVATOR:
            return "ELEVATOR";
    }
    return "UNKNOWN";
}

extern "C" dual_arm_ret_code_t dual_arm_set_group(dual_arm_control_api_t* api,
                                                  dual_arm_group_t group,
                                                  int enable) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->set_group(to_cpp_group(group), enable != 0));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_group_state(
    dual_arm_control_api_t* api, dual_arm_group_t group,
    dual_arm_state_t* out_states, size_t out_capacity, size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const auto [code, states] = api->cpp->get_group_state(to_cpp_group(group));
        if (out_count != nullptr) {
            *out_count = states.size();
        }
        if (code != dual_arm_v2_2_sdk::RetCode::SUCCESS) {
            return to_c_ret(code);
        }
        if (out_states == nullptr || out_capacity < states.size()) {
            set_last_error("Output buffer too small for group states");
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        for (size_t i = 0; i < states.size(); ++i) {
            out_states[i] = to_c_state(states[i]);
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_move_line(
    dual_arm_control_api_t* api, dual_arm_group_t group,
    const float* target_tool_pose, size_t pose_count, uint16_t speed_ratio,
    float acc_time) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (target_tool_pose == nullptr && pose_count > 0) {
        set_last_error("target_tool_pose is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(target_tool_pose,
                                        target_tool_pose + pose_count);
        return to_c_ret(api->cpp->move_line(to_cpp_group(group), values,
                                            speed_ratio, acc_time));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_tool_pose(
    dual_arm_control_api_t* api, dual_arm_group_t group, float* out_pose,
    size_t out_capacity, size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (out_count == nullptr) {
        set_last_error("out_count is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    *out_count = 0;
    try {
        const auto [code, pose] =
            api->cpp->get_tool_pose(to_cpp_group(group));
        if (code != dual_arm_v2_2_sdk::RetCode::SUCCESS) {
            return to_c_ret(code);
        }
        if (out_pose != nullptr) {
            const size_t n = std::min(out_capacity, pose.size());
            for (size_t i = 0; i < n; ++i) out_pose[i] = pose[i];
        }
        *out_count = pose.size();
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_move_joint(
    dual_arm_control_api_t* api, dual_arm_group_t group, const float* joints,
    size_t joint_count, int follow, uint8_t trajectory_mode, uint16_t radio,
    uint16_t speed_ratio) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (joints == nullptr && joint_count > 0) {
        set_last_error("joints is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(joints, joints + joint_count);
        return to_c_ret(api->cpp->move_joint(to_cpp_group(group), values, follow != 0,
                                             trajectory_mode, radio, speed_ratio));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_joint_target(
    dual_arm_control_api_t* api, dual_arm_group_t group, const float* joints,
    size_t joint_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (joints == nullptr && joint_count > 0) {
        set_last_error("joints is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(joints, joints + joint_count);
        return to_c_ret(
            api->cpp->set_joint_target(to_cpp_group(group), values));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_flush_joint_command(
    dual_arm_control_api_t* api, int follow, uint8_t trajectory_mode,
    uint16_t radio) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(
            api->cpp->flush_joint_command(follow != 0, trajectory_mode, radio));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" void dual_arm_reset_joint_target_cache(dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return;
    }
    try {
        api->cpp->reset_joint_target_cache();
    } catch (...) {
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_left_joint_target(
    dual_arm_control_api_t* api, const float* joints, size_t joint_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (joints == nullptr && joint_count > 0) {
        set_last_error("joints is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(joints, joints + joint_count);
        return to_c_ret(api->cpp->set_left_joint_target(values));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_right_joint_target(
    dual_arm_control_api_t* api, const float* joints, size_t joint_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (joints == nullptr && joint_count > 0) {
        set_last_error("joints is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(joints, joints + joint_count);
        return to_c_ret(api->cpp->set_right_joint_target(values));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_start_dual_arm_streaming(
    dual_arm_control_api_t* api, int follow, uint8_t trajectory_mode,
    uint16_t radio, int hz) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->start_dual_arm_streaming(
            follow != 0, trajectory_mode, radio, hz));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" void dual_arm_stop_dual_arm_streaming(dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return;
    }
    try {
        api->cpp->stop_dual_arm_streaming();
    } catch (...) {
    }
}

extern "C" int dual_arm_is_dual_arm_streaming(dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return 0;
    }
    try {
        return api->cpp->is_dual_arm_streaming() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

static dual_arm_ret_code_t set_tool_line_target_c(
    dual_arm_control_api_t* api, const float* tool_pose, size_t pose_count,
    dual_arm_v2_2_sdk::RetCode (dual_arm_v2_2_sdk::ControlApi::*setter)(
        const std::vector<float>&)) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (tool_pose == nullptr || pose_count != 7) {
        set_last_error("tool_pose must contain exactly 7 floats");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(tool_pose, tool_pose + pose_count);
        return to_c_ret((api->cpp->*setter)(values));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_left_tool_line_target(
    dual_arm_control_api_t* api, const float* tool_pose, size_t pose_count) {
    return set_tool_line_target_c(
        api, tool_pose, pose_count,
        &dual_arm_v2_2_sdk::ControlApi::set_left_tool_line_target);
}

extern "C" dual_arm_ret_code_t dual_arm_set_right_tool_line_target(
    dual_arm_control_api_t* api, const float* tool_pose, size_t pose_count) {
    return set_tool_line_target_c(
        api, tool_pose, pose_count,
        &dual_arm_v2_2_sdk::ControlApi::set_right_tool_line_target);
}

extern "C" dual_arm_ret_code_t dual_arm_flush_tool_line_command(
    dual_arm_control_api_t* api, uint16_t speed_ratio, float acc_time) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->flush_tool_line_command(speed_ratio, acc_time));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" void dual_arm_reset_tool_line_target_cache(
    dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return;
    }
    try {
        api->cpp->reset_tool_line_target_cache();
    } catch (...) {
    }
}

extern "C" dual_arm_ret_code_t dual_arm_start_dual_arm_tool_line_auto_flush(
    dual_arm_control_api_t* api, uint16_t speed_ratio, float acc_time,
    int coalesce_ms) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->start_dual_arm_tool_line_auto_flush(
            speed_ratio, acc_time, coalesce_ms));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" void dual_arm_stop_dual_arm_tool_line_auto_flush(
    dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return;
    }
    try {
        api->cpp->stop_dual_arm_tool_line_auto_flush();
    } catch (...) {
    }
}

extern "C" int dual_arm_is_dual_arm_tool_line_auto_flush(
    dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        return 0;
    }
    try {
        return api->cpp->is_dual_arm_tool_line_auto_flush() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_joint(dual_arm_control_api_t* api,
                                                  dual_arm_group_t group,
                                                  float* out_joints,
                                                  size_t out_capacity,
                                                  size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const auto [code, joints] = api->cpp->get_joint(to_cpp_group(group));
        if (code != dual_arm_v2_2_sdk::RetCode::SUCCESS) {
            if (out_count != nullptr) {
                *out_count = 0;
            }
            return to_c_ret(code);
        }
        if (!copy_float_vector(joints, out_joints, out_capacity, out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_joint_velocity(
    dual_arm_control_api_t* api, dual_arm_group_t group, float* out_velocities,
    size_t out_capacity, size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const auto [code, velocities] =
            api->cpp->get_joint_velocity(to_cpp_group(group));
        if (code != dual_arm_v2_2_sdk::RetCode::SUCCESS) {
            if (out_count != nullptr) {
                *out_count = 0;
            }
            return to_c_ret(code);
        }
        if (!copy_float_vector(velocities, out_velocities, out_capacity,
                               out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_joint_max_speed(
    dual_arm_control_api_t* api, dual_arm_group_t group, const float* limits,
    size_t limit_count, float* out_limits, size_t out_capacity,
    size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(limits, limits + limit_count);
        const std::vector<float> result =
            api->cpp->set_joint_max_speed(to_cpp_group(group), values);
        if (result.empty()) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        if (!copy_float_vector(result, out_limits, out_capacity, out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_joint_max_acc(
    dual_arm_control_api_t* api, dual_arm_group_t group, const float* limits,
    size_t limit_count, float* out_limits, size_t out_capacity,
    size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> values(limits, limits + limit_count);
        const std::vector<float> result =
            api->cpp->set_joint_max_acc(to_cpp_group(group), values);
        if (result.empty()) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        if (!copy_float_vector(result, out_limits, out_capacity, out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_joint_max_limit(
    dual_arm_control_api_t* api, dual_arm_group_t group, float* out_limits,
    size_t out_capacity, size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> result =
            api->cpp->get_joint_max_limit(to_cpp_group(group));
        if (result.empty()) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        if (!copy_float_vector(result, out_limits, out_capacity, out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_joint_min_limit(
    dual_arm_control_api_t* api, dual_arm_group_t group, float* out_limits,
    size_t out_capacity, size_t* out_count) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const std::vector<float> result =
            api->cpp->get_joint_min_limit(to_cpp_group(group));
        if (result.empty()) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        if (!copy_float_vector(result, out_limits, out_capacity, out_count)) {
            return DUAL_ARM_RET_CONTROLLER_ERROR;
        }
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_gripper_config(
    dual_arm_control_api_t* api, dual_arm_group_t group, uint16_t speed,
    uint16_t force, int block, int timeout_sec) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->set_gripper_config(
            to_cpp_group(group), speed, force, block != 0, timeout_sec));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_set_gripper_position(
    dual_arm_control_api_t* api, dual_arm_group_t group, uint16_t position,
    int block, int timeout_sec) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->set_gripper_position(
            to_cpp_group(group), position, block != 0, timeout_sec));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_get_gripper_state(
    dual_arm_control_api_t* api, dual_arm_group_t group,
    dual_arm_gripper_state_t* out_state) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    if (out_state == nullptr) {
        set_last_error("out_state is NULL");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        const auto [code, values] =
            api->cpp->get_gripper_state(to_cpp_group(group));
        if (code != dual_arm_v2_2_sdk::RetCode::SUCCESS) {
            return to_c_ret(code);
        }
        out_state->position = values.at("position");
        out_state->force = values.at("force");
        out_state->speed = values.at("speed");
        return DUAL_ARM_RET_SUCCESS;
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_start_teach_mode(
    dual_arm_control_api_t* api, int gravity_comp) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->start_teach_mode(gravity_comp != 0));
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}

extern "C" dual_arm_ret_code_t dual_arm_stop_teach_mode(
    dual_arm_control_api_t* api) {
    if (api == nullptr || api->cpp == nullptr) {
        set_last_error("Invalid API handle");
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
    try {
        return to_c_ret(api->cpp->stop_teach_mode());
    } catch (const std::exception& ex) {
        set_last_error(ex);
        return DUAL_ARM_RET_CONTROLLER_ERROR;
    }
}
