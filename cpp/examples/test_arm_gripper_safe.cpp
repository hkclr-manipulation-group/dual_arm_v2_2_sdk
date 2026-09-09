/**
 * @file test_arm_gripper_safe.cpp
 * @brief 安全流程演示：先开夹爪 -> 臂运动 -> 闭夹爪 -> 回 Home。
 *
 * 在文件顶部修改目标关节角（度）。需配置 robot.gripper。
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_arm_gripper_safe
 */

#include "control_api.h"
#include "example_common.h"

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
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

// ============ Edit target joint angles here (degrees, left arm J1..J7) ============
constexpr std::array<float, 7> kPose1JointsDeg = {15.0F, 10.0F, 12.0F, 8.0F, 10.0F, 5.0F, 5.0F};
constexpr std::array<float, 7> kPose2JointsDeg = {25.0F, 15.0F, 18.0F, 12.0F, 15.0F, 8.0F, 8.0F};
constexpr std::array<float, 7> kHomeJointsDeg = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

constexpr float kMaxSpeedDegS = 20.0F;
constexpr float kVelocityGuardDegS = 35.0F;

constexpr uint16_t kGripperOpen = 1000;
constexpr uint16_t kGripperClosed = 0;
constexpr uint16_t kGripperSpeed = 400;
constexpr uint16_t kGripperForce = 400;
constexpr int kGripperTimeoutSeconds = 10;
// ===================================================================================

constexpr float kMaxJointVelocity = kVelocityGuardDegS * kDegToRad;
constexpr float kTestMaxSpeed = kMaxSpeedDegS * kDegToRad;

bool wait_for_enter(const std::string& message) {
    std::cout << '\n' << message << '\n'
              << "Press Enter to continue, or Ctrl+D to abort..." << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) {
        std::cout << "\nTest aborted by user.\n";
        return false;
    }
    return true;
}

bool check_result(const char* operation, RetCode code) {
    std::cout << operation << ": " << ret_code_name(code) << '\n';
    return code == RetCode::SUCCESS;
}

std::vector<float> joints_deg_to_rad(const std::array<float, 7>& joints_deg) {
    std::vector<float> joints(joints_deg.size());
    for (std::size_t i = 0; i < joints_deg.size(); ++i)
        joints[i] = joints_deg[i] * kDegToRad;
    return joints;
}

void print_joints_deg(const char* label, const std::array<float, 7>& joints_deg) {
    std::cout << label << " (deg): ";
    for (std::size_t i = 0; i < joints_deg.size(); ++i)
        std::cout << "J" << i + 1 << '=' << joints_deg[i]
                  << (i + 1 < joints_deg.size() ? ", " : "\n");
}

void print_joints_rad(const std::vector<float>& joints) {
    std::cout << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < joints.size(); ++i)
        std::cout << "  J" << i + 1 << ": " << joints[i] * kRadToDeg << " deg\n";
}

bool print_gripper_state(ControlApi& api) {
    auto [code, state] = api.get_gripper_state(Group::LEFT_ARM);
    if (!check_result("get_gripper_state", code)) return false;
    std::cout << "  position: " << state.at("position") << " / 1000\n"
              << "  speed:    " << state.at("speed") << " / 1000\n"
              << "  force:    " << state.at("force") << " / 1000\n";
    return true;
}

bool wait_for_joint_state(ControlApi& api, std::vector<float>& joints,
                          int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    do {
        auto [code, value] = api.get_joint(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS) {
            joints = std::move(value);
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}

bool wait_for_gripper_state(ControlApi& api, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    do {
        auto [code, state] = api.get_gripper_state(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS) {
            std::cout << "Gripper online: position=" << state.at("position")
                      << ", speed=" << state.at("speed")
                      << ", force=" << state.at("force") << '\n';
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
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
                        std::cerr << "\nSAFETY STOP: J" << joint + 1
                                  << " reached " << velocity[joint] * kRadToDeg
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

bool wait_until_joint_target(ControlApi& api, const VelocityGuard& guard,
                             const std::vector<float>& target,
                             int timeout_seconds) {
    constexpr float kTolerance = 0.5F * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    int stable_samples = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        if (guard.triggered()) return false;

        auto [code, position] = api.get_joint(Group::LEFT_ARM);
        if (code != RetCode::SUCCESS || position.size() != target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        float largest_error = 0.0F;
        for (std::size_t i = 0; i < target.size(); ++i)
            largest_error =
                std::max(largest_error, std::abs(position[i] - target[i]));

        stable_samples =
            largest_error <= kTolerance ? stable_samples + 1 : 0;
        if (stable_samples >= 10) {
            std::cout << "Arm target reached.\n";
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Timed out waiting for the arm target.\n";
    return false;
}

bool move_arm_low_follow(ControlApi& api, const VelocityGuard& guard,
                         const std::vector<float>& joints,
                         const char* label, int timeout_seconds) {
    if (guard.triggered()) return false;
    if (!check_result(label, api.move_joint(Group::LEFT_ARM, joints, false)))
        return false;
    return wait_until_joint_target(api, guard, joints, timeout_seconds);
}

bool move_gripper(ControlApi& api, uint16_t position, const char* label) {
    if (!check_result(label, api.set_gripper_position(
                                  Group::LEFT_ARM, position, true,
                                  kGripperTimeoutSeconds)))
        return false;
    return print_gripper_state(api);
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::vector<float> current;

        const std::vector<float> pose1 = joints_deg_to_rad(kPose1JointsDeg);
        const std::vector<float> pose2 = joints_deg_to_rad(kPose2JointsDeg);
        const std::vector<float> home = joints_deg_to_rad(kHomeJointsDeg);

        std::cout << "Safe arm + gripper demo\n"
                  << "  - edit kPose1JointsDeg / kPose2JointsDeg / "
                     "kHomeJointsDeg at the top of this file\n"
                  << "  - each array is [J1, J2, J3, J4, J5, J6, J7] in degrees\n";
        print_joints_deg("Pose 1", kPose1JointsDeg);
        print_joints_deg("Pose 2", kPose2JointsDeg);
        print_joints_deg("Home", kHomeJointsDeg);

        std::cout << "\nWaiting for rt_control arm state...\n";
        if (!wait_for_joint_state(api, current, 3000)) {
            std::cerr << "No arm state received within 3 seconds.\n";
            return 1;
        }
        if (current.size() != pose1.size()) {
            std::cerr << "Expected " << pose1.size()
                      << " left-arm joints, got " << current.size() << ".\n";
            return 1;
        }
        std::cout << "Current left-arm joints:\n";
        print_joints_rad(current);

        std::cout << "\nWaiting for rt_control gripper state...\n";
        if (!wait_for_gripper_state(api, 3000)) {
            std::cerr << "No gripper state received within 3 seconds.\n";
            return 1;
        }

        VelocityGuard guard(api);

        if (!wait_for_enter(
                "Stage 1/6: configure gripper with moderate speed/force."))
            return 0;
        if (!check_result("set_gripper_config",
                          api.set_gripper_config(
                              Group::LEFT_ARM, kGripperSpeed, kGripperForce,
                              true, 2)))
            return 1;

        if (!wait_for_enter("Stage 2/6: open gripper before any arm motion."))
            return 0;
        if (!move_gripper(api, kGripperOpen, "set_gripper_position(open)"))
            return 1;

        if (!wait_for_enter("Stage 3/6: enable left arm and move to Pose 1."))
            return 0;
        if (!check_result("set_group(enable)",
                          api.set_group(Group::LEFT_ARM, true)))
            return 1;

        const std::vector<float> requested_speed(current.size(), kTestMaxSpeed);
        if (api.set_joint_max_speed(Group::LEFT_ARM, requested_speed).size() !=
            requested_speed.size()) {
            std::cerr << "set_joint_max_speed failed.\n";
            return 1;
        }
        std::cout << "set_joint_max_speed: " << kMaxSpeedDegS
                  << " deg/s for every left-arm joint\n";

        if (!move_arm_low_follow(api, guard, pose1, "move_joint(pose1)", 20))
            return 1;

        if (!wait_for_enter(
                "Stage 4/6: close gripper while the arm is stationary."))
            return 0;
        if (!move_gripper(api, kGripperClosed, "set_gripper_position(close)"))
            return 1;

        if (!wait_for_enter("Stage 5/6: move to Pose 2."))
            return 0;
        if (!move_arm_low_follow(api, guard, pose2, "move_joint(pose2)", 20))
            return 1;

        if (!wait_for_enter("Stage 6/6: open gripper and return to Home."))
            return 0;
        if (!move_gripper(api, kGripperOpen, "set_gripper_position(release)"))
            return 1;
        if (!move_arm_low_follow(api, guard, home, "move_joint(home)", 25))
            return 1;

        auto [joint_code, final_joints] = api.get_joint(Group::LEFT_ARM);
        if (!check_result("get_joint(final)", joint_code)) return 1;
        std::cout << "Final left-arm joints:\n";
        print_joints_rad(final_joints);
        if (!print_gripper_state(api)) return 1;

        std::cout << "Safe arm + gripper demo completed. "
                     "The arm remains enabled.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
