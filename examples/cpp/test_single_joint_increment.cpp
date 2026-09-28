/**
 * @file test_single_joint_increment.cpp
 * @brief 单臂单关节增量运动，用于校验 config_left/right.yaml 中 sign 方向是否正确。
 *
 * 前置条件：rt_control 已启动。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_single_joint_increment left 1 5
 *   ./test_single_joint_increment right 3 -3
 *   ./test_single_joint_increment left 2 10 --return
 *   ./test_single_joint_increment /path/to/cuarm_configuration/dual_v2_2 left 1 5
 *
 * sign 配置文件（修改后需重启 rt_control）：
 *   cuarm_rt_control/hardware_modules/realman_dual_arm/config_left.yaml
 *   cuarm_rt_control/hardware_modules/realman_dual_arm/config_right.yaml
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
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
using example::wait_for_joint_state;

namespace {

constexpr float kMaxSpeedDegS = 10.0F;
constexpr float kPositionToleranceDeg = 0.5F;
constexpr int kReachTimeoutSeconds = 20;

struct Options {
    std::string config_path;
    Group arm{Group::LEFT_ARM};
    int joint_index{0};  // 0-based
    float delta_deg{0.0F};
    bool return_to_start{false};
};

void print_usage(const char* prog) {
    std::cerr
        << "Usage:\n"
        << "  " << prog << " <left|right> <joint 1-7> <delta_deg> [--return]\n"
        << "  " << prog
        << " <config_prefix> <left|right> <joint 1-7> <delta_deg> [--return]\n\n"
        << "Examples:\n"
        << "  " << prog << " left 1 5       # left arm J1 +5 deg\n"
        << "  " << prog << " right 4 -3     # right arm J4 -3 deg\n"
        << "  " << prog << " left 2 10 --return\n";
}

bool parse_arm_token(const std::string& token, Group* arm) {
    if (token == "left" || token == "L" || token == "l") {
        *arm = Group::LEFT_ARM;
        return true;
    }
    if (token == "right" || token == "R" || token == "r") {
        *arm = Group::RIGHT_ARM;
        return true;
    }
    return false;
}

bool looks_like_config_prefix(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path / "config.yaml", ec);
}

bool parse_options(int argc, char* argv[], Options* opts) {
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--return") {
            opts->return_to_start = true;
            continue;
        }
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        }
        positional.push_back(arg);
    }

    if (positional.size() == 3) {
        opts->config_path = config_path_from_args(argc, argv);
    } else if (positional.size() == 4) {
        if (!looks_like_config_prefix(positional.front())) {
            std::cerr << "Config prefix not found: " << positional.front() << '\n';
            return false;
        }
        opts->config_path =
            std::filesystem::weakly_canonical(positional.front()).string();
        positional.erase(positional.begin());
    } else {
        print_usage(argv[0]);
        return false;
    }

    if (!parse_arm_token(positional[0], &opts->arm)) {
        std::cerr << "Invalid arm: " << positional[0]
                  << " (expected left or right)\n";
        return false;
    }

    opts->joint_index = std::atoi(positional[1].c_str()) - 1;
    if (opts->joint_index < 0 || opts->joint_index > 6) {
        std::cerr << "Invalid joint: " << positional[1]
                  << " (expected 1..7)\n";
        return false;
    }

    opts->delta_deg = std::strtof(positional[2].c_str(), nullptr);
    if (opts->delta_deg == 0.0F) {
        std::cerr << "Warning: delta_deg is 0; the arm will not move.\n";
    }
    return true;
}

void print_joint_deg(const char* label, const std::vector<float>& joints_rad) {
    std::cout << std::fixed << std::setprecision(3) << label << " (deg): [";
    for (std::size_t i = 0; i < joints_rad.size(); ++i) {
        std::cout << joints_rad[i] * kRadToDeg;
        if (i + 1 < joints_rad.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

bool wait_until_joint_reached(ControlApi& api, Group arm,
                              const std::vector<float>& target) {
    const float tolerance_rad = kPositionToleranceDeg * kDegToRad;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(kReachTimeoutSeconds);
    int stable = 0;

    while (std::chrono::steady_clock::now() <= deadline) {
        auto [code, current] = api.get_joint(arm);
        if (code != RetCode::SUCCESS || current.size() != target.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        float max_error = 0.0F;
        for (std::size_t i = 0; i < target.size(); ++i) {
            max_error = std::max(max_error, std::abs(current[i] - target[i]));
        }

        if (max_error <= tolerance_rad) {
            if (++stable >= 3) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

bool move_single_joint(ControlApi& api, Group arm, int joint_index,
                       const std::vector<float>& start_rad,
                       float delta_deg, const char* step_label) {
    std::vector<float> target = start_rad;
    target[static_cast<std::size_t>(joint_index)] +=
        delta_deg * kDegToRad;

    const auto min_limit = api.get_joint_min_limit(arm);
    const auto max_limit = api.get_joint_max_limit(arm);
    if (min_limit.size() == target.size() &&
        max_limit.size() == target.size()) {
        const float q = target[static_cast<std::size_t>(joint_index)];
        const float q_min = min_limit[static_cast<std::size_t>(joint_index)];
        const float q_max = max_limit[static_cast<std::size_t>(joint_index)];
        if (q < q_min || q > q_max) {
            std::cerr << step_label << ": target J" << joint_index + 1
                      << " = " << q * kRadToDeg << " deg is outside soft limit ["
                      << q_min * kRadToDeg << ", " << q_max * kRadToDeg
                      << "] deg\n";
            return false;
        }
    }

    std::cout << "\n==> " << step_label << '\n';
    std::cout << "    " << group_name(arm) << " J" << joint_index + 1
              << " command delta: " << std::showpos << delta_deg << std::noshowpos
              << " deg\n";

    const RetCode code = api.move_joint(arm, target, false);
    if (code != RetCode::SUCCESS) {
        std::cerr << "move_joint failed: " << ret_code_name(code) << '\n';
        return false;
    }

    if (!wait_until_joint_reached(api, arm, target)) {
        std::cout << "    WARNING: timed out after " << kReachTimeoutSeconds
                  << " s waiting to settle\n";
    }

    std::vector<float> actual;
    if (!wait_for_joint_state(api, arm, actual)) {
        std::cerr << "Failed to read joint feedback\n";
        return false;
    }

    print_joint_deg("    target", target);
    print_joint_deg("    actual", actual);

    const float cmd_delta =
        (target[static_cast<std::size_t>(joint_index)] -
         start_rad[static_cast<std::size_t>(joint_index)]) *
        kRadToDeg;
    const float fb_delta =
        (actual[static_cast<std::size_t>(joint_index)] -
         start_rad[static_cast<std::size_t>(joint_index)]) *
        kRadToDeg;
    std::cout << std::fixed << std::setprecision(3)
              << "    J" << joint_index + 1 << " feedback delta: " << std::showpos
              << fb_delta << std::noshowpos << " deg (command "
              << std::showpos << cmd_delta << std::noshowpos << " deg)\n";
    return true;
}

void print_sign_hint(Group arm, int joint_index) {
    const char* yaml = (arm == Group::LEFT_ARM)
                           ? "config_left.yaml"
                           : "config_right.yaml";
    std::cout
        << "\nSign 校验提示:\n"
        << "  - SDK 下发用户域 (MDH) 角度；rt_control 用 " << yaml
        << " 中 hardware.motor.sign 换算电机方向\n"
        << "  - 观察 J" << joint_index + 1
        << " 物理转动是否与命令增量同号；若相反，将该关节 sign 取反并重启 rt_control\n"
        << "  - 左臂 sign 参考: [1, 1, 1, -1, 1, -1, 1]\n"
        << "  - 右臂 sign 参考: [-1, 1, -1, -1, -1, 1, -1]\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    Options opts;
    if (!parse_options(argc, argv, &opts)) {
        return argc > 1 ? 1 : 0;
    }

    ControlApi api(opts.config_path);

    std::vector<float> start;
    if (!wait_for_joint_state(api, opts.arm, start)) {
        std::cerr << "No state received within 3 s. Is rt_control running?\n";
        return 1;
    }
    if (static_cast<int>(start.size()) <= opts.joint_index) {
        std::cerr << "Unexpected joint count: " << start.size() << '\n';
        return 1;
    }

    if (api.set_group(opts.arm, true) != RetCode::SUCCESS) {
        std::cerr << "Failed to enable " << group_name(opts.arm) << '\n';
        return 1;
    }

    const std::vector<float> speed(start.size(), kMaxSpeedDegS * kDegToRad);
    if (api.set_joint_max_speed(opts.arm, speed).size() != speed.size()) {
        std::cerr << "set_joint_max_speed failed\n";
        return 1;
    }

    std::cout << "Config: " << opts.config_path << '\n'
              << "Arm: " << group_name(opts.arm)
              << ", joint J" << opts.joint_index + 1
              << ", delta " << std::showpos << opts.delta_deg << std::noshowpos
              << " deg, max speed " << kMaxSpeedDegS << " deg/s\n";
    print_joint_deg("Start", start);

    if (!move_single_joint(api, opts.arm, opts.joint_index, start,
                           opts.delta_deg, "increment move")) {
        return 1;
    }

    print_sign_hint(opts.arm, opts.joint_index);

    if (opts.return_to_start) {
        if (!move_single_joint(api, opts.arm, opts.joint_index, start,
                               -opts.delta_deg, "return to start")) {
            return 1;
        }
    }

    return 0;
}
