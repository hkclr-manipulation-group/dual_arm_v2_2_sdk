/**
 * @file test_control_api.cpp
 * @brief SDK 交互式冒烟测试：组控制、关节读写、低/高跟随、速度限位。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_control_api          # 默认不主动运动
 *   ./test_control_api 0.01     # J1 偏移 +0.01 rad 的小幅运动测试
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
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
using dual_arm_v2_2_sdk::State;
using example::kConfigPath;
using example::kDegToRad;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr float kMaxJointVelocity = 30.0F * kDegToRad;

const char* state_name(State state) {
    switch (state) {
        case State::SHUTDOWN: return "SHUTDOWN";
        case State::OPERATIONAL: return "OPERATIONAL";
        case State::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

void print_joints(const std::vector<float>& joints) {
    constexpr float kRadToDeg = 57.29577951308232F;
    std::cout << std::fixed << std::setprecision(4);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        std::cout << "  J" << i + 1 << ": " << joints[i] << " rad ("
                  << joints[i] * kRadToDeg << " deg)\n";
    }
}

void print_joint_limits(const std::vector<float>& minimum,
                        const std::vector<float>& maximum) {
    constexpr float kRadToDeg = 57.29577951308232F;
    std::cout << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < minimum.size(); ++i) {
        std::cout << "  J" << i + 1 << ": ["
                  << minimum[i] << ", " << maximum[i] << "] rad  (["
                  << minimum[i] * kRadToDeg << ", "
                  << maximum[i] * kRadToDeg << "] deg)\n";
    }
}

bool check_result(const char* operation, RetCode code) {
    std::cout << operation << ": " << ret_code_name(code)
              << " (" << static_cast<int>(code) << ")\n";
    return code == RetCode::SUCCESS;
}

bool wait_for_enter(const std::string& message) {
    std::cout << "\n" << message << "\n"
              << "Press Enter to continue, or Ctrl+D to abort..." << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) {
        std::cout << "\nTest aborted by user.\n";
        return false;
    }
    return true;
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
        constexpr float kRadToDeg = 57.29577951308232F;
        std::vector<int> over_limit_count;

        while (!stop_.load()) {
            auto [code, velocity] = api_.get_joint_velocity(Group::LEFT_ARM);
            if (code == RetCode::SUCCESS) {
                if (over_limit_count.size() != velocity.size())
                    over_limit_count.assign(velocity.size(), 0);

                for (std::size_t i = 0; i < velocity.size(); ++i) {
                    if (std::abs(velocity[i]) <= kMaxJointVelocity) {
                        over_limit_count[i] = 0;
                        continue;
                    }

                    ++over_limit_count[i];
                    if (over_limit_count[i] < 3) continue;

                    if (!triggered_.exchange(true)) {
                        std::cerr << "\nSAFETY STOP: J" << i + 1
                                  << " velocity is " << velocity[i] * kRadToDeg
                                  << " deg/s, exceeding the 30 deg/s limit for "
                                     "3 consecutive samples.\n";

                        auto [position_code, position] =
                            api_.get_joint(Group::LEFT_ARM);
                        if (position_code == RetCode::SUCCESS) {
                            const RetCode hold_code = api_.move_joint(
                                Group::LEFT_ARM, position, true, 0, 0);
                            std::cerr << "Current-position hold command: "
                                      << ret_code_name(hold_code) << '\n';
                        } else {
                            std::cerr << "Could not read the current position; "
                                         "hold command was not sent.\n";
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

bool wait_until_target(ControlApi& api, const VelocityGuard& velocity_guard,
                       const std::vector<float>& target, int timeout_seconds) {
    constexpr float kPositionTolerance = 0.5F * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    int stable_samples = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        if (velocity_guard.triggered()) return false;

        auto [code, position] = api.get_joint(Group::LEFT_ARM);
        if (code != RetCode::SUCCESS || position.size() != target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        float largest_error = 0.0F;
        for (std::size_t i = 0; i < target.size(); ++i)
            largest_error = std::max(largest_error,
                                     std::abs(position[i] - target[i]));

        stable_samples = largest_error <= kPositionTolerance
                             ? stable_samples + 1
                             : 0;
        if (stable_samples >= 10) {
            std::cout << "Target reached.\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Timed out waiting for the target position.\n";
    return false;
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::vector<float> current;
        std::cout << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, current, 3000)) {
            std::cerr << "No state received within 3 seconds. Is rt_control running, "
                         "and do the UDP addresses/ports match config.yaml?\n";
            return 1;
        }

        std::cout << "Current left-arm joints:\n";
        print_joints(current);

        auto [state_code, states] = api.get_group_state(Group::LEFT_ARM);
        if (!check_result("get_group_state", state_code)) return 1;
        for (std::size_t i = 0; i < states.size(); ++i)
            std::cout << "  J" << i + 1 << ": " << state_name(states[i]) << '\n';

        const std::vector<float> minimum_position =
            api.get_joint_min_limit(Group::LEFT_ARM);
        const std::vector<float> maximum_position =
            api.get_joint_max_limit(Group::LEFT_ARM);
        if (minimum_position.size() != current.size() ||
            maximum_position.size() != current.size()) {
            std::cerr << "Failed to read left-arm joint position limits.\n";
            return 1;
        }
        std::cout << "Left-arm joint position limits:\n";
        print_joint_limits(minimum_position, maximum_position);

        const std::vector<float> target_10_deg(current.size(), 10.0F * kDegToRad);
        const std::vector<float> target_20_deg(current.size(), 20.0F * kDegToRad);
        const std::vector<float> target_0_deg(current.size(), 0.0F);
        std::cout << "Test sequence for every left-arm joint:\n"
                  << "  low-follow: current position -> 10 deg\n"
                  << "  high-follow direct: 10 deg -> 20 deg\n"
                  << "  high-follow 1 Hz filter: 20 deg -> 0 deg\n";
        std::cout << "Safety guard: if any left-arm joint exceeds 30 deg/s, "
                     "for 3 consecutive samples, hold the current position "
                     "and stop the test.\n";

        VelocityGuard velocity_guard(api);

        if (!wait_for_enter(
                "Stage 1/3: enable the left arm and execute low-follow "
                "point-to-point motion."))
            return 0;
        if (velocity_guard.triggered()) return 1;
        if (!check_result("set_group(enable)",
                          api.set_group(Group::LEFT_ARM, true)))
            return 1;

        constexpr float kTestMaximumSpeed = 20.0F * kDegToRad;
        const std::vector<float> requested_speed(current.size(),
                                                  kTestMaximumSpeed);
        const std::vector<float> applied_speed =
            api.set_joint_max_speed(Group::LEFT_ARM, requested_speed);
        if (applied_speed.size() != requested_speed.size()) {
            std::cerr << "set_joint_max_speed failed.\n";
            return 1;
        }
        std::cout << "set_joint_max_speed: 20 deg/s ("
                  << kTestMaximumSpeed << " rad/s) for every left-arm joint\n";

        constexpr float kTestMaximumAcceleration = 50.0F * kDegToRad;
        const std::vector<float> requested_acceleration(
            current.size(), kTestMaximumAcceleration);
        const std::vector<float> applied_acceleration =
            api.set_joint_max_acc(Group::LEFT_ARM, requested_acceleration);
        if (applied_acceleration.size() != requested_acceleration.size()) {
            std::cerr << "set_joint_max_acc failed.\n";
            return 1;
        }
        std::cout << "set_joint_max_acc: 50 deg/s^2 ("
                  << kTestMaximumAcceleration
                  << " rad/s^2) for every left-arm joint"
                     " [currently stored in SDK only]\n";

        if (!check_result("move_joint(low-follow)",
                          api.move_joint(Group::LEFT_ARM, target_10_deg, false)))
            return 1;
        if (!wait_until_target(api, velocity_guard, target_10_deg, 15))
            return 1;

        if (!wait_for_enter(
                "Stage 2/3: execute high-follow direct streaming for 2 seconds "
                "at 100 Hz."))
            return 0;
        if (velocity_guard.triggered()) return 1;

        for (int i = 0; i < 200; ++i) {
            if (velocity_guard.triggered()) return 1;
            const float ratio = static_cast<float>(i + 1) / 200.0F;
            std::vector<float> target(current.size(),
                                      (10.0F + 10.0F * ratio) * kDegToRad);
            const RetCode code = api.move_joint(
                Group::LEFT_ARM, target, true, 0, 0);
            if (code != RetCode::SUCCESS) {
                check_result("move_joint(high-follow, direct)", code);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check_result("move_joint(high-follow, direct)", RetCode::SUCCESS);
        if (!wait_until_target(api, velocity_guard, target_20_deg, 3))
            return 1;

        if (!wait_for_enter(
                "Stage 3/3: execute high-follow low-frequency filtering for "
                "2 seconds at 100 Hz (radio=999, cutoff=1 Hz)."))
            return 0;
        if (velocity_guard.triggered()) return 1;

        for (int i = 0; i < 200; ++i) {
            if (velocity_guard.triggered()) return 1;
            const float ratio = static_cast<float>(i + 1) / 200.0F;
            std::vector<float> target(current.size(),
                                      (20.0F - 20.0F * ratio) * kDegToRad);
            const RetCode code = api.move_joint(
                Group::LEFT_ARM, target, true, 2, 999);
            if (code != RetCode::SUCCESS) {
                check_result("move_joint(high-follow, 1 Hz filter)", code);
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        check_result("move_joint(high-follow, 1 Hz filter)", RetCode::SUCCESS);
        if (velocity_guard.triggered()) return 1;
        if (!wait_until_target(api, velocity_guard, target_0_deg, 10))
            return 1;

        auto [joint_code, final_joints] = api.get_joint(Group::LEFT_ARM);
        if (!check_result("get_joint(final)", joint_code)) return 1;
        std::cout << "Final left-arm joints:\n";
        print_joints(final_joints);

        // Keep the arm enabled. Disabling it automatically at process exit could
        // make a gravity-loaded arm fall, so that action must remain explicit.
        std::cout << "Smoke test completed. The arm remains enabled.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
