/**
 * @file test_gripper_api.cpp
 * @brief 单夹爪 API 交互测试（需 config.yaml 中配置 robot.gripper）。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_gripper_api
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
using example::kConfigPath;
using example::ret_code_name;
using example::wait_for_enter;

namespace {

bool print_state(ControlApi& api) {
    auto [code, state] = api.get_gripper_state(Group::LEFT_ARM);
    std::cout << "get_gripper_state: " << ret_code_name(code) << '\n';
    if (code != RetCode::SUCCESS) return false;
    std::cout << "  position: " << state["position"] << " / 1000\n"
              << "  speed:    " << state["speed"] << " / 1000\n"
              << "  force:    " << state["force"] << " / 1000\n";
    return true;
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::cout << "Waiting for rt_control gripper state...\n";
        bool received = false;
        RetCode last_gripper_code = RetCode::RECEIVE_FAILED;
        for (int attempt = 0; attempt < 150; ++attempt) {
            auto [code, state] = api.get_gripper_state(Group::LEFT_ARM);
            last_gripper_code = code;
            if (code == RetCode::SUCCESS) {
                received = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!received) {
            auto [joint_code, joints] = api.get_joint(Group::LEFT_ARM);
            if (joint_code != RetCode::SUCCESS) {
                std::cerr
                    << "No rt_control state received within 3 seconds.\n"
                    << "Check that dual_v2_2_rt_control is still running. "
                       "If it exited during startup, inspect its terminal for "
                       "CXT CAN / RMD gripper initialization errors.\n";
            } else if (last_gripper_code == RetCode::CONTROLLER_ERROR) {
                std::cerr
                    << "rt_control state is arriving, but the gripper reports "
                       "a controller error.\n"
                    << "Check the CXT CAN device, CAN channel 0, motor ID 8, "
                       "power, and CAN wiring.\n";
            } else {
                std::cerr
                    << "rt_control state is arriving, but it contains no "
                       "left-gripper state.\n"
                    << "Restart the newly built dual_v2_2_rt_control and make "
                       "sure config.yaml contains robot.gripper.\n";
            }
            return 1;
        }
        if (!print_state(api)) return 1;

        if (!wait_for_enter(
                "Configure speed=200 (72 deg/s output shaft), force=500 "
                "(0.5 Nm)."))
            return 0;
        RetCode code = api.set_gripper_config(
            Group::LEFT_ARM, 1000, 500, true, 2);
        std::cout << "set_gripper_config: " << ret_code_name(code) << '\n';
        if (code != RetCode::SUCCESS) return 1;

        if (!wait_for_enter("Open the gripper to position 1000.")) return 0;
        code = api.set_gripper_position(
            Group::LEFT_ARM, 1000, true, 10);
        std::cout << "set_gripper_position(open): "
                  << ret_code_name(code) << '\n';
        if (code != RetCode::SUCCESS || !print_state(api)) return 1;

        if (!wait_for_enter(
                "Close the gripper to position 0; contact torque can finish the command early."))
            return 0;
        code = api.set_gripper_position(
            Group::LEFT_ARM, 0, true, 10);
        std::cout << "set_gripper_position(close): "
                  << ret_code_name(code) << '\n';
        if (code != RetCode::SUCCESS || !print_state(api)) return 1;

        std::cout << "Gripper test completed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
