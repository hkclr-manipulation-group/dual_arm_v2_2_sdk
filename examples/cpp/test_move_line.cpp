/**
 * @file test_move_line.cpp
 * @brief MoveL demo: 与 examples/c/test_move_line.c 对齐。
 *
 * 流程（启动时左右臂同时使能，头部 disable）：
 *   左臂：初始关节位姿 -> MoveL +Z -> MoveL 回程 -> 收敛等待 -> 关节回零
 *   右臂：初始关节位姿 -> MoveL -Z -> MoveL 回程 -> 收敛等待 -> 关节回零
 *
 * 用法：./test_move_line [config_prefix_path]
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
using example::group_name;
using example::kDegToRad;
using example::kRadToDeg;
using example::ret_code_name;

namespace {

constexpr float kInitialJointsDeg[7] = {10.0F, 10.0F, 10.0F, 45.0F,
                                        5.0F, 5.0F, 5.0F};
constexpr float kLeftZOffsetM = 0.30F;
constexpr float kRightZOffsetM = -0.20F;
constexpr uint16_t kSpeedRatio = 15;
constexpr float kMoveLineAccTimeS = 8.0F;
constexpr float kPosToleranceM = 0.005F;
constexpr float kJointToleranceRad = 0.5F * kDegToRad;
constexpr float kJointVelToleranceRadS = 1.0F * kDegToRad;
constexpr float kMaxSpeedDegS = 12.0F;
constexpr float kHomeMaxSpeedDegS = 8.0F;
constexpr int kMoveTimeoutMs = 40000;
constexpr int kInitialReachTimeoutMs = 40000;
constexpr int kHomeTimeoutMs = 40000;
constexpr int kSettleTimeoutMs = 15000;
constexpr int kPollMs = 50;
constexpr int kSettleStableCycles = 5;
constexpr int kPoseStableCycles = 3;

std::vector<float> initial_joints_rad() {
    std::vector<float> joints(7);
    for (size_t i = 0; i < 7; ++i) joints[i] = kInitialJointsDeg[i] * kDegToRad;
    return joints;
}

void print_joints_deg(const char* label, const std::vector<float>& joints_rad) {
    std::cout << label << " (deg): ";
    for (size_t i = 0; i < joints_rad.size(); ++i) {
        std::cout << "J" << (i + 1) << "=" << std::fixed << std::setprecision(1)
                  << joints_rad[i] * kRadToDeg;
        if (i + 1 < joints_rad.size()) std::cout << ", ";
    }
    std::cout << '\n';
}

void print_tool_pose(const char* label, const std::vector<float>& pose) {
    if (pose.size() < 7) return;
    std::cout << label << '\n';
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  position (m): " << pose[0] << ", " << pose[1] << ", "
              << pose[2] << '\n';
    std::cout << "  quaternion:   " << pose[3] << ", " << pose[4] << ", "
              << pose[5] << ", " << pose[6] << '\n';
}

std::vector<float> offset_z(const std::vector<float>& pose, float dz) {
    std::vector<float> target = pose;
    if (target.size() >= 3) target[2] += dz;
    return target;
}

bool verify_joint_pose_deg(const char* label,
                           const std::vector<float>& joints_rad) {
    bool ok = true;
    std::cout << label << '\n';
    for (size_t i = 0; i < joints_rad.size() && i < 7; ++i) {
        const float actual_deg = joints_rad[i] * kRadToDeg;
        const float err_deg = std::abs(actual_deg - kInitialJointsDeg[i]);
        if (err_deg > 0.5F) {
            std::cout << "  ERROR J" << (i + 1) << ": target "
                      << kInitialJointsDeg[i] << " deg, actual "
                      << actual_deg << " deg (err " << err_deg << ")\n";
            ok = false;
        }
    }
    return ok;
}

bool wait_for_motion_settle(ControlApi& api, Group group, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    std::vector<float> prev;
    for (;;) {
        auto [code, current] = api.get_joint(group);
        if (code == RetCode::SUCCESS && current.size() == 7) {
            if (!prev.empty()) {
                float max_vel = 0.0F;
                for (size_t i = 0; i < 7; ++i) {
                    max_vel = std::max(
                        max_vel,
                        std::abs(current[i] - prev[i]) / (kPollMs * 0.001F));
                }
                if (max_vel <= kJointVelToleranceRadS) {
                    if (++stable >= kSettleStableCycles) return true;
                } else {
                    stable = 0;
                }
            }
            prev = current;
        }
        if (std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

bool wait_for_joint_target(ControlApi& api, Group group,
                           const std::vector<float>& target, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, current] = api.get_joint(group);
        if (code != RetCode::SUCCESS || current.size() != target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            continue;
        }
        float max_err = 0.0F;
        for (size_t i = 0; i < target.size(); ++i)
            max_err = std::max(max_err, std::abs(current[i] - target[i]));
        if (max_err <= kJointToleranceRad) {
            if (++stable >= 3) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

bool configure_both_arms(ControlApi& api) {
    if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::HEAD, false) != RetCode::SUCCESS) {
        std::cout << "Failed to enable left + right arms (head disabled).\n";
        return false;
    }
    std::cout << "Left + right arms enabled, head disabled.\n";
    return true;
}

bool setup_arm_speed_deg(ControlApi& api, Group group, float max_speed_deg_s) {
    const std::vector<float> speed(7, max_speed_deg_s * kDegToRad);
    const auto applied = api.set_joint_max_speed(group, speed);
    if (applied.size() != 7) {
        std::cout << "set_joint_max_speed(" << group_name(group) << ") failed\n";
        return false;
    }
    return true;
}

bool setup_arm_speed(ControlApi& api, Group group) {
    return setup_arm_speed_deg(api, group, kMaxSpeedDegS);
}

bool move_to_initial_pose(ControlApi& api, Group group) {
    const std::vector<float> target = initial_joints_rad();

    std::cout << "\n==> move_joint initial pose (" << group_name(group) << ")\n";
    print_joints_deg("  target", target);

    const RetCode code = api.move_joint(group, target, false);
    std::cout << "  move_joint: " << ret_code_name(code) << '\n';
    if (code != RetCode::SUCCESS) return false;

    if (!wait_for_joint_target(api, group, target, kInitialReachTimeoutMs)) {
        std::cout << "  WARNING: initial pose not reached within "
                  << kInitialReachTimeoutMs << " ms\n";
        return false;
    }

    auto [joint_code, actual] = api.get_joint(group);
    if (joint_code != RetCode::SUCCESS || actual.size() != 7) {
        std::cout << "  ERROR: failed to read joints after initial move\n";
        return false;
    }

    print_joints_deg("  actual", actual);
    if (!verify_joint_pose_deg("  initial pose check:", actual)) {
        std::cout << "  ERROR: initial pose verification failed for "
                  << group_name(group) << '\n';
        return false;
    }
    std::cout << "==> initial pose done (" << group_name(group) << ")\n";
    return true;
}

bool hold_current_joint_pose(ControlApi& api, Group group) {
    auto [code, current] = api.get_joint(group);
    if (code != RetCode::SUCCESS || current.size() != 7) {
        std::cout << "  ERROR: failed to read current joints ("
                  << group_name(group) << ")\n";
        return false;
    }

    std::cout << "\n==> hold current joint pose (" << group_name(group)
              << ", TaskLine -> Joint handoff)\n";
    print_joints_deg("  hold", current);

    const RetCode move_code = api.move_joint(group, current, false);
    std::cout << "  move_joint: " << ret_code_name(move_code) << '\n';
    if (move_code != RetCode::SUCCESS) return false;

    if (!wait_for_joint_target(api, group, current, kInitialReachTimeoutMs)) {
        std::cout << "  WARNING: hold pose not reached within "
                  << kInitialReachTimeoutMs << " ms\n";
        return false;
    }
    return true;
}

bool prepare_for_go_home(ControlApi& api, Group group) {
    std::cout << "\n==> prepare for go_home (" << group_name(group) << ")\n";

    if (!wait_for_motion_settle(api, group, kSettleTimeoutMs)) {
        std::cout << "  WARNING: motion did not settle within "
                  << kSettleTimeoutMs << " ms\n";
        return false;
    }
    std::cout << "  motion settled (joint vel <= "
              << kJointVelToleranceRadS * kRadToDeg << " deg/s)\n";

    if (!hold_current_joint_pose(api, group)) return false;

    if (!wait_for_motion_settle(api, group, kSettleTimeoutMs)) {
        std::cout << "  WARNING: hold pose did not settle within "
                  << kSettleTimeoutMs << " ms\n";
        return false;
    }
    std::cout << "  ready for go_home\n";
    return true;
}

bool go_home(ControlApi& api, Group group) {
    const std::vector<float> zero(7, 0.0F);

    if (!setup_arm_speed_deg(api, group, kHomeMaxSpeedDegS)) return false;

    std::cout << "\n==> go home (" << group_name(group)
              << ", all joints -> 0)\n";
    const RetCode code = api.move_joint(group, zero, false);
    std::cout << "  move_joint: " << ret_code_name(code) << '\n';
    if (code != RetCode::SUCCESS) return false;

    if (!wait_for_joint_target(api, group, zero, kHomeTimeoutMs)) {
        std::cout << "  WARNING: home pose not reached within "
                  << kHomeTimeoutMs << " ms\n";
        return false;
    }

    auto [joint_code, actual] = api.get_joint(group);
    if (joint_code == RetCode::SUCCESS && actual.size() == 7)
        print_joints_deg("  actual", actual);
    std::cout << "==> home done (" << group_name(group) << ")\n";
    return setup_arm_speed(api, group);
}

bool wait_tool_pose_z(ControlApi& api, Group group, float target_z,
                      float tolerance_m, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, pose] = api.get_tool_pose(group);
        if (code == RetCode::SUCCESS && pose.size() >= 3 &&
            std::abs(pose[2] - target_z) <= tolerance_m) {
            if (++stable >= kPoseStableCycles) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

bool run_move_line(ControlApi& api, Group group,
                   const std::vector<float>& target) {
    const RetCode code =
        api.move_line(group, target, kSpeedRatio, kMoveLineAccTimeS);
    std::cout << "move_line(" << group_name(group)
              << "): " << ret_code_name(code) << '\n';
    if (code != RetCode::SUCCESS) return false;
    if (target.size() < 3) return true;
    if (wait_tool_pose_z(api, group, target[2], kPosToleranceM, kMoveTimeoutMs))
        return true;
    std::cout << "Warning: ACK ok but ToolPose Z did not reach target within "
              << kMoveTimeoutMs << " ms\n";
    return false;
}

bool demo_movel_and_return(ControlApi& api, Group group, float z_offset_m) {
    auto [pose_code, current] = api.get_tool_pose(group);
    if (pose_code != RetCode::SUCCESS || current.size() < 7) {
        std::cout << "get_tool_pose failed for " << group_name(group) << '\n';
        return false;
    }

    print_tool_pose("Current tool pose:", current);

    const std::vector<float> forward = offset_z(current, z_offset_m);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Target (Z " << std::showpos << z_offset_m << " m):\n"
              << std::noshowpos;
    print_tool_pose(" ", forward);
    if (!run_move_line(api, group, forward)) return false;

    auto [after_code, after_forward] = api.get_tool_pose(group);
    if (after_code != RetCode::SUCCESS || after_forward.size() < 7) {
        std::cout << "get_tool_pose failed after MoveL\n";
        return false;
    }
    print_tool_pose("After MoveL:", after_forward);

    const std::vector<float> back = offset_z(after_forward, -z_offset_m);
    print_tool_pose("Target (cartesian return):", back);
    if (!run_move_line(api, group, back)) return false;

    auto [final_code, final_pose] = api.get_tool_pose(group);
    if (final_code == RetCode::SUCCESS)
        print_tool_pose("After cartesian return:", final_pose);
    return true;
}

bool run_arm_cycle(ControlApi& api, Group group, const char* label,
                   float z_offset_m) {
    std::cout << "\n========================================\n";
    std::cout << "=== " << label << " ===\n";
    std::cout << "========================================\n";

    if (!move_to_initial_pose(api, group)) return false;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\n--- MoveL Z " << std::showpos << z_offset_m
              << " m, then cartesian return ---\n" << std::noshowpos;
    if (!demo_movel_and_return(api, group, z_offset_m)) return false;

    if (!prepare_for_go_home(api, group)) return false;

    if (!go_home(api, group)) return false;
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    ControlApi api(config_path_from_args(argc, argv));

    if (!configure_both_arms(api) ||
        !setup_arm_speed(api, Group::LEFT_ARM) ||
        !setup_arm_speed(api, Group::RIGHT_ARM)) {
        return 1;
    }

    if (!run_arm_cycle(api, Group::LEFT_ARM, "Left arm cycle", kLeftZOffsetM))
        return 1;

    if (!run_arm_cycle(api, Group::RIGHT_ARM, "Right arm cycle", kRightZOffsetM))
        return 1;

    std::cout << "\nDone.\n";
    return 0;
}
