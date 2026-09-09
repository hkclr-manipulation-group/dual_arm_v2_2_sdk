/**
 * @file test_control_api_various_command_frequency.cpp
 * @brief 高跟随模式下不同发送频率（20~30 Hz）的行为对比测试。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_control_api_various_command_frequency
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::kConfigPath;
using example::kDegToRad;
using example::kPi;
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr float kMaxJointVelocity = 30.0F * kDegToRad;

// 33 ms is about 30 Hz, 50 ms is 20 Hz. Both high-follow tests use the same
// deterministic pattern, making their behavior easier to compare.
constexpr std::array<int, 12> kVariablePeriodsMs = {
    33, 50, 36, 47, 34, 44, 39, 49, 35, 42, 38, 46};

bool wait_for_enter(const std::string& message) {
    std::cout << '\n' << message << '\n'
              << "Press Enter to continue, or Ctrl+D to abort..." << std::flush;
    std::string input;
    return static_cast<bool>(std::getline(std::cin, input));
}

class VelocityGuard {
public:
    explicit VelocityGuard(ControlApi& api) : api_(api) {
        thread_ = std::thread(&VelocityGuard::run, this);
    }

    ~VelocityGuard() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
    }

    bool triggered() const { return triggered_.load(); }

private:
    void run() {
        std::vector<int> over_limit_count;
        while (!stop_.load()) {
            auto [code, velocity] = api_.get_joint_velocity(Group::LEFT_ARM);
            if (code == RetCode::SUCCESS) {
                if (over_limit_count.size() != velocity.size())
                    over_limit_count.assign(velocity.size(), 0);

                for (std::size_t joint = 0; joint < velocity.size(); ++joint) {
                    if (std::abs(velocity[joint]) <= kMaxJointVelocity) {
                        over_limit_count[joint] = 0;
                        continue;
                    }
                    if (++over_limit_count[joint] < 3) continue;

                    if (!triggered_.exchange(true)) {
                        std::cerr << "\nSAFETY STOP: J" << joint + 1 << " reached "
                                  << velocity[joint] * kRadToDeg
                                  << " deg/s for 3 consecutive samples.\n";
                        auto [position_code, position] =
                            api_.get_joint(Group::LEFT_ARM);
                        if (position_code == RetCode::SUCCESS) {
                            const RetCode hold = api_.move_joint(
                                Group::LEFT_ARM, position, true, 0, 0);
                            std::cerr << "Current-position hold command: "
                                      << ret_code_name(hold) << '\n';
                        }
                    }
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    ControlApi& api_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> triggered_{false};
    std::thread thread_;
};

bool wait_until_target(ControlApi& api, const VelocityGuard& guard,
                       const std::vector<float>& target, int timeout_seconds) {
    constexpr float kTolerance = 0.5F * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    int stable_samples = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        if (guard.triggered()) return false;
        auto [code, position] = api.get_joint(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS && position.size() == target.size()) {
            float maximum_error = 0.0F;
            for (std::size_t joint = 0; joint < target.size(); ++joint)
                maximum_error = std::max(
                    maximum_error, std::abs(position[joint] - target[joint]));
            stable_samples =
                maximum_error <= kTolerance ? stable_samples + 1 : 0;
            if (stable_samples >= 10) {
                std::cout << "Target reached.\n";
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Timed out waiting for the target position.\n";
    return false;
}

bool send_variable_frequency_trajectory(
    ControlApi& api, const VelocityGuard& guard, std::size_t joint_count,
    float start_deg, float finish_deg, uint8_t trajectory_mode,
    uint16_t radio, const char* label) {
    constexpr float kDurationSeconds = 8.0F;
    const auto start_time = std::chrono::steady_clock::now();
    std::size_t period_index = 0;
    auto next_report = start_time;

    while (true) {
        if (guard.triggered()) return false;
        const auto now = std::chrono::steady_clock::now();
        const float elapsed =
            std::chrono::duration<float>(now - start_time).count();
        const float progress = std::min(elapsed / kDurationSeconds, 1.0F);

        // This source trajectory is smooth in real time. Direct mode exposes
        // irregular command arrivals; filter mode smooths their steps.
        const float smooth_progress =
            0.5F - 0.5F * std::cos(kPi * progress);
        const float command_deg =
            start_deg + (finish_deg - start_deg) * smooth_progress;
        const std::vector<float> target(joint_count, command_deg * kDegToRad);

        const RetCode code = api.move_joint(
            Group::LEFT_ARM, target, true, trajectory_mode, radio);
        if (code != RetCode::SUCCESS) {
            std::cerr << label << " send failed: " << ret_code_name(code) << '\n';
            return false;
        }

        if (now >= next_report) {
            auto [position_code, position] = api.get_joint(Group::LEFT_ARM);
            std::cout << std::fixed << std::setprecision(2)
                      << label << "  t=" << elapsed
                      << " s, period=" << kVariablePeriodsMs[period_index]
                      << " ms, command=" << command_deg << " deg";
            if (position_code == RetCode::SUCCESS && !position.empty())
                std::cout << ", actual J1=" << position.front() * kRadToDeg
                          << " deg";
            std::cout << '\n';
            next_report += std::chrono::seconds(1);
        }

        if (progress >= 1.0F) break;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(kVariablePeriodsMs[period_index]));
        period_index = (period_index + 1) % kVariablePeriodsMs.size();
    }
    return true;
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::vector<float> current;
        std::cout << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, current)) {
            std::cerr << "No rt_control state received within 3 seconds.\n";
            return 1;
        }

        std::cout << "Current left-arm position:\n" << std::fixed
                  << std::setprecision(3);
        for (std::size_t joint = 0; joint < current.size(); ++joint)
            std::cout << "  J" << joint + 1 << ": "
                      << current[joint] * kRadToDeg << " deg\n";

        std::cout << "\nTiming pattern: 33-50 ms per command (about 30-20 Hz).\n"
                  << "Safety: hold position and stop if any joint exceeds "
                     "30 deg/s for 3 consecutive samples.\n";

        const std::vector<float> target_10(current.size(), 10.0F * kDegToRad);
        const std::vector<float> target_20(current.size(), 20.0F * kDegToRad);
        VelocityGuard guard(api);

        if (!wait_for_enter(
                "Stage 1/4: low-follow point-to-point, current -> 10 deg."))
            return 0;
        RetCode code = api.set_group(Group::LEFT_ARM, true);
        std::cout << "set_group(enable): " << ret_code_name(code) << '\n';
        if (code != RetCode::SUCCESS) return 1;
        code = api.move_joint(Group::LEFT_ARM, target_10, false);
        std::cout << "move_joint(low-follow): " << ret_code_name(code)
                  << " (the call only waits for command acknowledgement)\n";
        if (code != RetCode::SUCCESS ||
            !wait_until_target(api, guard, target_10, 20))
            return 1;

        if (!wait_for_enter(
                "Stage 2/4: high-follow direct, 10 -> 20 deg with variable frequency."))
            return 0;
        if (!send_variable_frequency_trajectory(
                api, guard, current.size(), 10.0F, 20.0F, 0, 0, "direct"))
            return 1;
        if (!wait_until_target(api, guard, target_20, 5)) return 1;

        if (!wait_for_enter(
                "Stage 3/4: low-follow reset, 20 -> 10 deg."))
            return 0;
        code = api.move_joint(Group::LEFT_ARM, target_10, false);
        std::cout << "move_joint(low-follow reset): "
                  << ret_code_name(code) << '\n';
        if (code != RetCode::SUCCESS ||
            !wait_until_target(api, guard, target_10, 20))
            return 1;

        if (!wait_for_enter(
                "Stage 4/4: high-follow 1 Hz filter, 10 -> 20 deg with the same variable frequency."))
            return 0;
        if (!send_variable_frequency_trajectory(
                api, guard, current.size(), 10.0F, 20.0F, 2, 999, "filter"))
            return 1;
        if (!wait_until_target(api, guard, target_20, 15)) return 1;

        std::cout << "Variable-frequency test completed. "
                     "The arm remains enabled.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
