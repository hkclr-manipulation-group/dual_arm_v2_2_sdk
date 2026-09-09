/**
 * @file go_home.cpp
 * @brief 双臂回零：左/右臂各 7 关节置 0，头部保持当前角度。
 *
 * 前置条件：
 *   - rt_control 已启动，且 config.yaml 路径与 kConfigPath 一致
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./go_home
 */

#include "control_api.h"
#include "example_common.h"

#include <iostream>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::kConfigPath;
using example::wait_for_joint_state;

int main() {
    ControlApi api(kConfigPath);

    std::vector<float> probe;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
        std::cerr << "No state received within 3 s. Is rt_control running?\n";
        return 1;
    }

    if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS) {
        std::cerr << "Failed to enable arms\n";
        return 1;
    }

    std::vector<float> head;
    std::vector<float> left;
    if (!wait_for_joint_state(api, Group::HEAD, head) ||
        !wait_for_joint_state(api, Group::LEFT_ARM, left)) {
        std::cerr << "Failed to read joint state\n";
        return 1;
    }

    const std::vector<float> zero(left.size(), 0.0F);
    std::vector<float> all;
    all.insert(all.end(), zero.begin(), zero.end());
    all.insert(all.end(), head.begin(), head.end());
    all.insert(all.end(), zero.begin(), zero.end());

    if (api.move_joint(Group::ALL, all, false) != RetCode::SUCCESS) {
        std::cerr << "move_joint failed\n";
        return 1;
    }

    std::cout << "Both arms moving to home (arm joints zero, head unchanged)\n";
    return 0;
}
