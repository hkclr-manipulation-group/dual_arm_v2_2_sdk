/**
 * @file test_dual_gripper_api.cpp
 * @brief 双臂夹爪 API 交互测试（左/右各一个 gripper）。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_dual_gripper_api
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::group_name;
using example::kConfigPath;
using example::ret_code_name;
using example::wait_for_enter;

namespace {

constexpr uint16_t kGripperOpen = 1000;
constexpr uint16_t kGripperClosed = 0;
constexpr uint16_t kGripperSpeed = 400;
constexpr uint16_t kGripperForce = 500;
constexpr int kGripperTimeoutSeconds = 10;

bool print_state(ControlApi& api, Group group) {
    auto [code, state] = api.get_gripper_state(group);
    std::cout << group_name(group) << " get_gripper_state: "
              << ret_code_name(code) << '\n';
    if (code != RetCode::SUCCESS) return false;
    std::cout << "  position: " << state.at("position") << " / 1000\n"
              << "  speed:    " << state.at("speed") << " / 1000\n"
              << "  force:    " << state.at("force") << " / 1000\n";
    return true;
}

bool wait_for_gripper_state(ControlApi& api, Group group, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    do {
        auto [code, state] = api.get_gripper_state(group);
        if (code == RetCode::SUCCESS) {
            std::cout << group_name(group) << " gripper online: position="
                      << state.at("position") << ", speed="
                      << state.at("speed") << ", force="
                      << state.at("force") << '\n';
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

bool configure_gripper(ControlApi& api, Group group) {
    RetCode code = api.set_gripper_config(
        group, kGripperSpeed, kGripperForce, true, 2);
    std::cout << group_name(group) << " set_gripper_config: "
              << ret_code_name(code) << '\n';
    return code == RetCode::SUCCESS;
}

bool move_gripper(ControlApi& api, Group group, uint16_t position,
                  const char* label) {
    RetCode code = api.set_gripper_position(
        group, position, true, kGripperTimeoutSeconds);
    std::cout << group_name(group) << ' ' << label << ": "
              << ret_code_name(code) << '\n';
    return code == RetCode::SUCCESS && print_state(api, group);
}

bool test_one_gripper(ControlApi& api, Group group, const char* stage_prefix) {
    if (!configure_gripper(api, group)) return false;

    std::string open_prompt = std::string(stage_prefix) +
                              ": open " + group_name(group) + " gripper.";
    if (!wait_for_enter(open_prompt.c_str())) return false;
    if (!move_gripper(api, group, kGripperOpen, "set_gripper_position(open)"))
        return false;

    std::string close_prompt = std::string(stage_prefix) +
                               ": close " + group_name(group) + " gripper.";
    if (!wait_for_enter(close_prompt.c_str())) return false;
    if (!move_gripper(api, group, kGripperClosed, "set_gripper_position(close)"))
        return false;

    return true;
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::cout << "Dual gripper test (LEFT + RIGHT)\n"
                  << "Prerequisites:\n"
                  << "  1. dual_v2_2_rt_control is running with with_gripper config\n"
                  << "  2. CAN wiring matches gripper_left/right.yaml:\n"
                  << "     - single box dual channel: left (0,0), right (0,1)\n"
                  << "     - dual box: left (0,0), right (1,0)\n"
                  << "  3. Only one gripper connected is also supported\n";

        const bool left_online =
            wait_for_gripper_state(api, Group::LEFT_ARM, 3000);
        const bool right_online =
            wait_for_gripper_state(api, Group::RIGHT_ARM, 3000);

        if (!left_online && !right_online) {
            std::cerr << "No gripper state received within 3 seconds.\n"
                      << "Check rt_control startup logs and CAN wiring.\n";
            return 1;
        }
        if (!left_online) {
            std::cout << "LEFT gripper unavailable; continuing with RIGHT only.\n";
        }
        if (!right_online) {
            std::cout << "RIGHT gripper unavailable; continuing with LEFT only.\n";
        }

        if (left_online) {
            if (!wait_for_enter("Stage 1: test LEFT gripper open/close."))
                return 0;
            if (!test_one_gripper(api, Group::LEFT_ARM, "Stage 1")) return 1;
        }

        if (right_online) {
            if (!wait_for_enter("Stage 2: test RIGHT gripper open/close."))
                return 0;
            if (!test_one_gripper(api, Group::RIGHT_ARM, "Stage 2")) return 1;
        }

        if (left_online && right_online) {
            if (!wait_for_enter(
                    "Stage 3: open both grippers, then close both grippers."))
                return 0;

            RetCode left_code = api.set_gripper_position(
                Group::LEFT_ARM, kGripperOpen, true, kGripperTimeoutSeconds);
            RetCode right_code = api.set_gripper_position(
                Group::RIGHT_ARM, kGripperOpen, true, kGripperTimeoutSeconds);
            std::cout << "Both open: LEFT=" << ret_code_name(left_code)
                      << ", RIGHT=" << ret_code_name(right_code) << '\n';
            if (left_code != RetCode::SUCCESS || right_code != RetCode::SUCCESS ||
                !print_state(api, Group::LEFT_ARM) ||
                !print_state(api, Group::RIGHT_ARM))
                return 1;

            left_code = api.set_gripper_position(
                Group::LEFT_ARM, kGripperClosed, true, kGripperTimeoutSeconds);
            right_code = api.set_gripper_position(
                Group::RIGHT_ARM, kGripperClosed, true, kGripperTimeoutSeconds);
            std::cout << "Both close: LEFT=" << ret_code_name(left_code)
                      << ", RIGHT=" << ret_code_name(right_code) << '\n';
            if (left_code != RetCode::SUCCESS || right_code != RetCode::SUCCESS ||
                !print_state(api, Group::LEFT_ARM) ||
                !print_state(api, Group::RIGHT_ARM))
                return 1;
        }

        std::cout << "Dual gripper test completed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
