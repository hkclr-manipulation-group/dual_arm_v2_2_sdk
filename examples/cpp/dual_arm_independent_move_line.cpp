/**
 * @file dual_arm_independent_move_line.cpp
 * @brief 低跟随双臂独立 MoveL：set_left/right_tool_line_target + auto flush。
 *
 * 不用 move_line(ALL, 14)。SDK 后台线程将左右 Tool 目标合并在**同一 UDP 包**
 * 里发 kTaskLine，并阻塞等待 send_task_with_ack（低跟随 quintic MoveL）。
 *
 * 流程：使能 L/R、关头 → start_dual_arm_tool_line_auto_flush →
 *       连续 set_left / set_right（不同目标）→ 等待到位 → 再 set 回起始。
 *
 * 用法：./dual_arm_independent_move_line
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::config_path_from_args;
using example::kDegToRad;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr float kLeftDeltaZM = 0.15F;
constexpr float kRightDeltaZM = -0.12F;
constexpr uint16_t kSpeedRatio = 25;
constexpr float kAccTimeS = 3.0F;
constexpr int kCoalesceMs = 15;
constexpr float kPosToleranceM = 0.008F;
constexpr float kMaxSpeedDegS = 15.0F;
constexpr int kPollMs = 50;
constexpr int kStableCycles = 4;

std::vector<float> offset_z(const std::vector<float>& pose, float dz) {
    std::vector<float> out = pose;
    if (out.size() >= 3) out[2] += dz;
    return out;
}

float position_error_m(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() < 3 || b.size() < 3) return 1e9F;
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool setup_both_arm_speed(ControlApi& api) {
    const std::vector<float> speed(7, kMaxSpeedDegS * kDegToRad);
    return api.set_joint_max_speed(Group::LEFT_ARM, speed).size() == 7 &&
           api.set_joint_max_speed(Group::RIGHT_ARM, speed).size() == 7;
}

bool wait_both_tool_targets(ControlApi& api,
                            const std::vector<float>& left_target,
                            const std::vector<float>& right_target,
                            int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [l_code, left_now] = api.get_tool_pose(Group::LEFT_ARM);
        auto [r_code, right_now] = api.get_tool_pose(Group::RIGHT_ARM);
        if (l_code != RetCode::SUCCESS || r_code != RetCode::SUCCESS ||
            left_now.size() < 7 || right_now.size() < 7) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            continue;
        }
        if (position_error_m(left_now, left_target) <= kPosToleranceM &&
            position_error_m(right_now, right_target) <= kPosToleranceM) {
            if (++stable >= kStableCycles) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

/** 连续 set 左右目标，让 coalesce 合并成一次低跟随 MoveL。 */
bool submit_both_targets(ControlApi& api, const std::vector<float>& left,
                         const std::vector<float>& right) {
    if (api.set_left_tool_line_target(left) != RetCode::SUCCESS) return false;
    if (api.set_right_tool_line_target(right) != RetCode::SUCCESS) return false;
    return true;
}

int move_timeout_ms() {
    return static_cast<int>((kAccTimeS + 2.0F) * 1000.0F);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        ControlApi api(config_path_from_args(argc, argv));

        std::vector<float> probe;
        if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
            std::cerr << "No rt_control state within 3 s.\n";
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::HEAD, false) != RetCode::SUCCESS) {
            std::cerr << "Failed to enable L/R (head disabled for MoveL).\n";
            return 1;
        }
        if (!setup_both_arm_speed(api)) {
            std::cerr << "set_joint_max_speed failed.\n";
            return 1;
        }

        auto [l0_code, left_start] = api.get_tool_pose(Group::LEFT_ARM);
        auto [r0_code, right_start] = api.get_tool_pose(Group::RIGHT_ARM);
        if (l0_code != RetCode::SUCCESS || r0_code != RetCode::SUCCESS ||
            left_start.size() != 7 || right_start.size() != 7) {
            std::cerr << "get_tool_pose failed.\n";
            return 1;
        }

        if (api.start_dual_arm_tool_line_auto_flush(kSpeedRatio, kAccTimeS,
                                                    kCoalesceMs) !=
            RetCode::SUCCESS) {
            std::cerr << "start_dual_arm_tool_line_auto_flush failed\n";
            return 1;
        }

        std::cout << "Low-follow dual MoveL: set_left/right + auto flush\n"
                  << "  coalesce_ms=" << kCoalesceMs
                  << "  speed_ratio=" << kSpeedRatio
                  << "  acc_time=" << kAccTimeS << " s\n"
                  << "  LEFT dZ=" << kLeftDeltaZM << " m, RIGHT dZ="
                  << kRightDeltaZM << " m\n";

        const std::vector<float> left_fwd = offset_z(left_start, kLeftDeltaZM);
        const std::vector<float> right_fwd =
            offset_z(right_start, kRightDeltaZM);

        std::cout << "\n==> Forward: set_left then set_right (one coalesced MoveL)\n";
        if (!submit_both_targets(api, left_fwd, right_fwd)) {
            std::cerr << "set_*_tool_line_target failed\n";
            api.stop_dual_arm_tool_line_auto_flush();
            return 1;
        }
        if (!wait_both_tool_targets(api, left_fwd, right_fwd, move_timeout_ms())) {
            std::cerr << "Timeout waiting forward MoveL\n";
            api.stop_dual_arm_tool_line_auto_flush();
            return 1;
        }
        std::cout << "  forward OK\n";

        std::cout << "\n==> Return: set_left/right back to start ToolPose\n";
        if (!submit_both_targets(api, left_start, right_start)) {
            std::cerr << "set_*_tool_line_target failed\n";
            api.stop_dual_arm_tool_line_auto_flush();
            return 1;
        }
        if (!wait_both_tool_targets(api, left_start, right_start,
                                    move_timeout_ms())) {
            std::cerr << "Timeout waiting return MoveL\n";
            api.stop_dual_arm_tool_line_auto_flush();
            return 1;
        }
        std::cout << "  return OK\n";

        api.stop_dual_arm_tool_line_auto_flush();
        std::cout << "\nDone.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal: " << error.what() << '\n';
        return 1;
    }
}
