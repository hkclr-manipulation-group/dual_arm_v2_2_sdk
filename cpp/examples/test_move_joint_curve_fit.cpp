/**
 * @file test_move_joint_curve_fit.cpp
 * @brief move_joint 曲线拟合模式（trajectory_mode=1）演示。
 *
 * 映射到 rt_control kSinglePoint + kQuintic；InterpolationAccTime = 10ms + radio/100。
 * 流式发送时命令间隔应 >= acc_time。
 *
 * Part 1：单次曲线拟合，radio=20（快）vs radio=100（平滑），仅 J1 运动
 * Part 2：低速路点流（间隔 = 1.2 * acc_time）
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./test_move_joint_curve_fit
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
using example::kConfigPath;
using example::kDegToRad;
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {
constexpr float kTestMaxSpeedDegS = 50.0F;
constexpr float kTestMaxSpeedRadS = kTestMaxSpeedDegS * kDegToRad;
// Guard slightly above the configured speed limit.
constexpr float kGuardVelocityDegS = kTestMaxSpeedDegS + 5.0F;
constexpr float kGuardVelocityRadS = kGuardVelocityDegS * kDegToRad;
constexpr float kPositionTolerance = 0.5F * kDegToRad;

constexpr float kJ1PrepOffsetDeg = 15.0F;
constexpr float kJ1MidOffsetDeg = 25.0F;
constexpr float kJ1FinalOffsetDeg = 35.0F;
constexpr float kStreamStepDeg = 5.0F;
constexpr uint16_t kFastRadio = 20;
constexpr uint16_t kSmoothRadio = 100;

float curve_fit_acc_time(uint16_t radio) {
    return 0.01F + static_cast<float>(radio) / 100.0F;
}

int curve_fit_interval_ms(uint16_t radio) {
    return static_cast<int>(curve_fit_acc_time(radio) * 1200.0F);
}

bool wait_for_enter(const std::string& message) {
    std::cout << '\n' << message << '\n'
              << "Press Enter to continue, or Ctrl+D to abort..." << std::flush;
    std::string input;
    return static_cast<bool>(std::getline(std::cin, input));
}

bool configure_arm_speed(ControlApi& api, std::size_t joint_count) {
    const std::vector<float> requested_speed(joint_count, kTestMaxSpeedRadS);
    const std::vector<float> applied_speed =
        api.set_joint_max_speed(Group::LEFT_ARM, requested_speed);
    if (applied_speed.size() != requested_speed.size()) {
        std::cerr << "set_joint_max_speed failed.\n";
        return false;
    }
    std::cout << "set_joint_max_speed: " << kTestMaxSpeedDegS
              << " deg/s for every left-arm joint\n";
    return true;
}

std::vector<float> j1_offset_target(const std::vector<float>& reference,
                                    float j1_offset_deg) {
    std::vector<float> target = reference;
    if (!target.empty())
        target[0] += j1_offset_deg * kDegToRad;
    return target;
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
                    if (std::abs(velocity[joint]) <= kGuardVelocityRadS) {
                        over_limit_count[joint] = 0;
                        continue;
                    }
                    if (++over_limit_count[joint] < 3) continue;

                    if (!triggered_.exchange(true)) {
                        std::cerr << "\nSAFETY STOP: J" << joint + 1 << " reached "
                                  << velocity[joint] * kRadToDeg
                                  << " deg/s (guard limit "
                                  << kGuardVelocityDegS << " deg/s).\n";
                        auto [position_code, position] =
                            api_.get_joint(Group::LEFT_ARM);
                        if (position_code == RetCode::SUCCESS) {
                            const RetCode hold = api_.move_joint(
                                Group::LEFT_ARM, position, true, 0, 0);
                            std::cerr << "Direct hold at current position: "
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

float max_joint_error(const std::vector<float>& target,
                      const std::vector<float>& actual) {
    float maximum_error = 0.0F;
    const std::size_t count = std::min(target.size(), actual.size());
    for (std::size_t joint = 0; joint < count; ++joint)
        maximum_error = std::max(
            maximum_error, std::abs(actual[joint] - target[joint]));
    return maximum_error;
}

void print_motion_sample(const char* label, float elapsed_seconds,
                         const std::vector<float>& target,
                         const std::vector<float>& position,
                         const std::vector<float>& velocity) {
    std::cout << std::fixed << std::setprecision(2)
              << label << "  t=" << elapsed_seconds << " s";
    if (!position.empty())
        std::cout << ", actual J1=" << position.front() * kRadToDeg << " deg";
    if (!target.empty())
        std::cout << ", target J1=" << target.front() * kRadToDeg << " deg";
    if (!velocity.empty())
        std::cout << ", vel J1=" << velocity.front() * kRadToDeg << " deg/s";
    if (!position.empty() && !target.empty())
        std::cout << ", err J1="
                  << (position.front() - target.front()) * kRadToDeg << " deg";
    std::cout << '\n';
}

bool wait_until_target(ControlApi& api, const VelocityGuard& guard,
                       const std::vector<float>& target, int timeout_seconds,
                       const char* label = nullptr, bool print_progress = false) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    const auto start_time = std::chrono::steady_clock::now();
    auto next_report = start_time;
    int stable_samples = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        if (guard.triggered()) return false;

        const auto now = std::chrono::steady_clock::now();
        auto [code, position] = api.get_joint(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS && position.size() == target.size()) {
            const float error = max_joint_error(target, position);
            stable_samples =
                error <= kPositionTolerance ? stable_samples + 1 : 0;

            if (print_progress && label && now >= next_report) {
                auto [velocity_code, velocity] =
                    api.get_joint_velocity(Group::LEFT_ARM);
                const float elapsed =
                    std::chrono::duration<float>(now - start_time).count();
                print_motion_sample(
                    label, elapsed, target, position,
                    velocity_code == RetCode::SUCCESS ? velocity
                                                      : std::vector<float>{});
                next_report += std::chrono::milliseconds(200);
            }

            if (stable_samples >= 10) {
                if (label) {
                    const float elapsed =
                        std::chrono::duration<float>(now - start_time).count();
                    std::cout << label << " reached target in "
                              << std::fixed << std::setprecision(2)
                              << elapsed << " s.\n";
                } else {
                    std::cout << "Target reached.\n";
                }
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Timed out waiting for the target position.\n";
    return false;
}

bool move_low_follow(ControlApi& api, const VelocityGuard& guard,
                     const std::vector<float>& target, const char* label,
                     int timeout_seconds) {
    std::cout << label << '\n';
    const RetCode code = api.move_joint(Group::LEFT_ARM, target, false);
    std::cout << "  move_joint(low-follow): " << ret_code_name(code) << '\n';
    return code == RetCode::SUCCESS &&
           wait_until_target(api, guard, target, timeout_seconds);
}

bool move_curve_fit_single(ControlApi& api, const VelocityGuard& guard,
                           const std::vector<float>& target, uint16_t radio,
                           const char* label, int timeout_seconds) {
    const float acc_time = curve_fit_acc_time(radio);
    std::cout << label << " radio=" << radio
              << ", acc_time=" << std::fixed << std::setprecision(3)
              << acc_time << " s (single command)\n";

    const RetCode code =
        api.move_joint(Group::LEFT_ARM, target, true, 1, radio);
    if (code != RetCode::SUCCESS) {
        std::cerr << "  move_joint(curve-fit): " << ret_code_name(code)
                  << '\n';
        return false;
    }

    return wait_until_target(api, guard, target, timeout_seconds, label,
                             true);
}

bool stream_curve_fit_waypoints(ControlApi& api, const VelocityGuard& guard,
                                const std::vector<float>& home_pose,
                                float start_offset_deg, float finish_offset_deg,
                                float step_deg, uint16_t radio,
                                const char* label) {
    const float acc_time = curve_fit_acc_time(radio);
    const int interval_ms = curve_fit_interval_ms(radio);
    std::cout << label << " radio=" << radio
              << ", acc_time=" << std::fixed << std::setprecision(3)
              << acc_time << " s, waypoint interval=" << interval_ms
              << " ms\n";

    std::vector<float> waypoints_deg;
    for (float deg = start_offset_deg; deg <= finish_offset_deg + 1e-3F;
         deg += step_deg)
        waypoints_deg.push_back(deg);

    for (std::size_t index = 0; index < waypoints_deg.size(); ++index) {
        if (guard.triggered()) return false;

        const std::vector<float> target =
            j1_offset_target(home_pose, waypoints_deg[index]);
        const RetCode code =
            api.move_joint(Group::LEFT_ARM, target, true, 1, radio);
        if (code != RetCode::SUCCESS) {
            std::cerr << "  waypoint " << index + 1 << "/"
                      << waypoints_deg.size() << " send failed: "
                      << ret_code_name(code) << '\n';
            return false;
        }

        auto [position_code, position] = api.get_joint(Group::LEFT_ARM);
        auto [velocity_code, velocity] = api.get_joint_velocity(Group::LEFT_ARM);
        std::cout << std::fixed << std::setprecision(2)
                  << "  waypoint " << index + 1 << "/"
                  << waypoints_deg.size() << ": J1 offset cmd="
                  << waypoints_deg[index] << " deg";
        if (position_code == RetCode::SUCCESS && !position.empty())
            std::cout << ", actual J1=" << position.front() * kRadToDeg
                      << " deg";
        if (velocity_code == RetCode::SUCCESS && !velocity.empty())
            std::cout << ", vel J1=" << velocity.front() * kRadToDeg
                      << " deg/s";
        std::cout << '\n';

        if (index + 1 < waypoints_deg.size())
            std::this_thread::sleep_for(
                std::chrono::milliseconds(interval_ms));
    }

    const std::vector<float> final_target =
        j1_offset_target(home_pose, finish_offset_deg);
    return wait_until_target(api, guard, final_target, 20, label, true);
}

}  // namespace

int main() {
    try {
        ControlApi api(kConfigPath);
        std::vector<float> home_pose;
        std::cout << "move_joint curve-fit mode test (trajectory_mode=1)\n"
                  << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, home_pose)) {
            std::cerr << "No rt_control state received within 3 seconds.\n";
            return 1;
        }

        std::cout << "Home pose (only J1 will move relative to this):\n"
                  << std::fixed << std::setprecision(3);
        for (std::size_t joint = 0; joint < home_pose.size(); ++joint)
            std::cout << "  J" << joint + 1 << ": "
                      << home_pose[joint] * kRadToDeg << " deg\n";

        std::cout << "\nMotion plan: J1 offset "
                  << kJ1PrepOffsetDeg << " -> " << kJ1MidOffsetDeg << " -> "
                  << kJ1FinalOffsetDeg << " deg; other joints fixed.\n"
                  << "low-follow uses config.yaml panel.general.acc_time "
                     "(recommend >= 1 s).\n"
                  << "Safety guard: direct hold if any joint exceeds "
                  << kGuardVelocityDegS << " deg/s for 3 consecutive samples.\n";

        const std::vector<float> target_prep =
            j1_offset_target(home_pose, kJ1PrepOffsetDeg);
        const std::vector<float> target_mid =
            j1_offset_target(home_pose, kJ1MidOffsetDeg);
        const std::vector<float> target_final =
            j1_offset_target(home_pose, kJ1FinalOffsetDeg);
        VelocityGuard guard(api);

        if (!wait_for_enter(
                "Stage 1/6: enable left arm, set max speed, J1 -> +15 deg "
                "(low-follow)."))
            return 0;
        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS)
            return 1;
        if (!configure_arm_speed(api, home_pose.size()))
            return 1;
        if (!move_low_follow(api, guard, target_prep,
                             "Prepare: J1 +15 deg", 25))
            return 1;

        if (!wait_for_enter(
                "Stage 2/6: curve-fit FAST radio=20 (acc~210 ms), "
                "J1 +15 -> +25 deg (10 deg). Watch: quick snap."))
            return 0;
        if (!move_curve_fit_single(
                api, guard, target_mid, kFastRadio, "curve-fit-fast", 20))
            return 1;

        if (!wait_for_enter("Stage 3/6: reset J1 to +15 deg (low-follow)."))
            return 0;
        if (!move_low_follow(api, guard, target_prep, "Reset to J1 +15 deg",
                             25))
            return 1;

        if (!wait_for_enter(
                "Stage 4/6: curve-fit SMOOTH radio=100 (acc~1.01 s), "
                "J1 +15 -> +35 deg (20 deg). Watch: slow smooth arc."))
            return 0;
        if (!move_curve_fit_single(
                api, guard, target_final, kSmoothRadio, "curve-fit-smooth",
                30))
            return 1;

        if (!wait_for_enter(
                "Stage 5/6: reset to +15 deg, then curve-fit waypoint stream "
                "radio=100 (5 deg steps, interval ~1.2 s). Watch: stepped creep."))
            return 0;
        if (!move_low_follow(api, guard, target_prep, "Reset to J1 +15 deg",
                             25))
            return 1;
        if (!stream_curve_fit_waypoints(
                api, guard, home_pose, kJ1PrepOffsetDeg, kJ1FinalOffsetDeg,
                kStreamStepDeg, kSmoothRadio, "curve-fit-stream"))
            return 1;

        if (!wait_for_enter("Stage 6/6: return to home pose (low-follow)."))
            return 0;
        if (!move_low_follow(api, guard, home_pose, "Return home", 30))
            return 1;

        auto [final_code, final_joints] = api.get_joint(Group::LEFT_ARM);
        if (final_code == RetCode::SUCCESS && !final_joints.empty()) {
            std::cout << "Final left-arm J1: "
                      << final_joints.front() * kRadToDeg << " deg\n";
        }

        std::cout << "Curve-fit mode test completed. The arm remains enabled.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
