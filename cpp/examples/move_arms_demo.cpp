/**
 * @file move_arms_demo.cpp
 * @brief 单臂 / 双臂运动演示：左臂 -> 右臂 -> 双臂 -> 回零。
 *
 * 在文件顶部修改 kLeftOffsetDeg / kRightOffsetDeg（相对当前位姿的偏移，单位：度）。
 * 每步会等待关节到位并打印误差报告。
 *
 * 前置条件：rt_control 已启动。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./move_arms_demo
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::kConfigPath;
using example::kDegToRad;
using example::kRadToDeg;
using example::wait_for_joint_state;

namespace {

// ============ Edit here: per-joint offset from current pose (deg, J1..J7) ============
constexpr std::array<float, 7> kLeftOffsetDeg  = {10, 10, 10, 10, 10, 10, 10};
constexpr std::array<float, 7> kRightOffsetDeg = {10, 10, 10, 10, 10, 10, 10};
constexpr float kMaxSpeedDegS = 20.0F;
constexpr float kPositionToleranceDeg = 0.5F;
constexpr int kReachTimeoutSeconds = 15;
constexpr int kPauseBeforeBothSeconds = 2;
// ====================================================================================

std::vector<float> current_plus_offset(const std::vector<float>& current,
                                       const std::array<float, 7>& offset_deg) {
    std::vector<float> target = current;
    for (int i = 0; i < 7 && i < static_cast<int>(current.size()); ++i)
        target[i] += offset_deg[i] * kDegToRad;
    return target;
}

float max_joint_error(const std::vector<float>& target,
                      const std::vector<float>& actual) {
    float max_error = 0.0F;
    const std::size_t n = std::min(target.size(), actual.size());
    for (std::size_t i = 0; i < n; ++i)
        max_error = std::max(max_error, std::abs(actual[i] - target[i]));
    return max_error;
}

void print_arm_report(const char* arm_label, const std::vector<float>& target,
                      const std::vector<float>& actual) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  " << arm_label << " actual (deg):  [";
    for (std::size_t i = 0; i < actual.size(); ++i) {
        std::cout << actual[i] * kRadToDeg;
        if (i + 1 < actual.size()) std::cout << ", ";
    }
    std::cout << "]\n  " << arm_label << " target (deg): [";
    for (std::size_t i = 0; i < target.size(); ++i) {
        std::cout << target[i] * kRadToDeg;
        if (i + 1 < target.size()) std::cout << ", ";
    }
    std::cout << "]\n  " << arm_label << " error   (deg): [";
    const std::size_t n = std::min(target.size(), actual.size());
    float max_err = 0.0F;
    for (std::size_t i = 0; i < n; ++i) {
        const float err = (actual[i] - target[i]) * kRadToDeg;
        max_err = std::max(max_err, std::abs(err));
        std::cout << err;
        if (i + 1 < n) std::cout << ", ";
    }
    std::cout << "]\n  " << arm_label << " max |error| = " << max_err
              << " deg"
              << (max_err <= kPositionToleranceDeg ? "  [OK]" : "  [NOT REACHED]")
              << '\n';
}

bool wait_until_settled(ControlApi& api, bool check_left, bool check_right,
                        const std::vector<float>& left_target,
                        const std::vector<float>& right_target) {
    const float tolerance_rad = kPositionToleranceDeg * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(kReachTimeoutSeconds);
    int stable = 0;

    while (std::chrono::steady_clock::now() <= deadline) {
        auto [left_code, left_current] = api.get_joint(Group::LEFT_ARM);
        auto [right_code, right_current] = api.get_joint(Group::RIGHT_ARM);

        if ((check_left && left_code != RetCode::SUCCESS) ||
            (check_right && right_code != RetCode::SUCCESS)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (check_left && left_current.size() != left_target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (check_right && right_current.size() != right_target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        float max_error = 0.0F;
        if (check_left)
            max_error = std::max(max_error, max_joint_error(left_target, left_current));
        if (check_right)
            max_error = std::max(max_error, max_joint_error(right_target, right_current));

        if (max_error <= tolerance_rad) {
            if (++stable >= 3) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}

bool move_and_report(ControlApi& api, Group group, const std::vector<float>& command,
                     bool check_left, bool check_right,
                     const std::vector<float>& left_target,
                     const std::vector<float>& right_target,
                     const char* label) {
    std::cout << "\n==> " << label << '\n';
    if (api.move_joint(group, command, false) != RetCode::SUCCESS) {
        std::cerr << "move_joint failed\n";
        return false;
    }

    const bool reached = wait_until_settled(
        api, check_left, check_right, left_target, right_target);
    if (!reached)
        std::cout << "  WARNING: timed out after " << kReachTimeoutSeconds
                  << "s waiting to settle\n";

    std::vector<float> left_actual;
    std::vector<float> right_actual;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, left_actual) ||
        !wait_for_joint_state(api, Group::RIGHT_ARM, right_actual)) {
        std::cerr << "Failed to read joint feedback after move\n";
        return false;
    }

    if (check_left) print_arm_report("LEFT ", left_target, left_actual);
    if (check_right) print_arm_report("RIGHT", right_target, right_actual);
    if (!check_left && !check_right) {
        std::cout << "  (no reach check for this step)\n";
    }

    return true;
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);

        std::vector<float> left_now;
        std::vector<float> right_now;
        std::vector<float> head_now;
        if (!wait_for_joint_state(api, Group::LEFT_ARM, left_now) ||
            !wait_for_joint_state(api, Group::RIGHT_ARM, right_now) ||
            !wait_for_joint_state(api, Group::HEAD, head_now)) {
            std::cerr << "No rt_control state within 3 seconds.\n";
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS) {
            std::cerr << "Failed to enable arms.\n";
            return 1;
        }

        const std::vector<float> speed(7, kMaxSpeedDegS * kDegToRad);
        if (api.set_joint_max_speed(Group::LEFT_ARM, speed).size() != speed.size() ||
            api.set_joint_max_speed(Group::RIGHT_ARM, speed).size() != speed.size()) {
            std::cerr << "set_joint_max_speed failed.\n";
            return 1;
        }

        const std::vector<float> left_target =
            current_plus_offset(left_now, kLeftOffsetDeg);
        const std::vector<float> right_target =
            current_plus_offset(right_now, kRightOffsetDeg);

        if (!move_and_report(api, Group::LEFT_ARM, left_target,
                             true, false, left_target, right_target,
                             "Move LEFT arm only"))
            return 1;
        if (!move_and_report(api, Group::RIGHT_ARM, right_target,
                             false, true, left_target, right_target,
                             "Move RIGHT arm only"))
            return 1;

        std::cout << "\nPause " << kPauseBeforeBothSeconds
                  << "s, then move both arms together from current pose...\n";
        std::this_thread::sleep_for(std::chrono::seconds(kPauseBeforeBothSeconds));

        if (!wait_for_joint_state(api, Group::LEFT_ARM, left_now) ||
            !wait_for_joint_state(api, Group::RIGHT_ARM, right_now) ||
            !wait_for_joint_state(api, Group::HEAD, head_now)) {
            std::cerr << "Failed to read joint state before BOTH move.\n";
            return 1;
        }

        const std::vector<float> left_both =
            current_plus_offset(left_now, kLeftOffsetDeg);
        const std::vector<float> right_both =
            current_plus_offset(right_now, kRightOffsetDeg);

        std::vector<float> all;
        all.insert(all.end(), left_both.begin(), left_both.end());
        all.insert(all.end(), head_now.begin(), head_now.end());
        all.insert(all.end(), right_both.begin(), right_both.end());
        if (!move_and_report(api, Group::ALL, all,
                             true, true, left_both, right_both,
                             "Move BOTH arms together"))
            return 1;

        const std::vector<float> zero(left_now.size(), 0.0F);
        std::vector<float> home;
        home.insert(home.end(), zero.begin(), zero.end());
        home.insert(home.end(), head_now.begin(), head_now.end());
        home.insert(home.end(), zero.begin(), zero.end());
        if (!move_and_report(api, Group::ALL, home,
                             true, true, zero, zero,
                             "Go home (both arms zero)"))
            return 1;

        std::cout << "\nDemo done.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
