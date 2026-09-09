/**
 * @file teach_replay.cpp
 * @brief 示教点回放：循环播放关节空间轨迹，Ctrl+C 停止。
 *
 * 流程：
 *   1. 等待 rt_control 状态
 *   2. 启用双臂
 *   3. 从文件加载示教点并循环回放（低跟随 move_joint）
 *
 * 示教文件格式（逗号分隔，单位：度）：
 *   idx, left_j1..j7, [left_gripper,] right_j1..j7, [right_gripper]
 *   支持 15 列（无夹爪）或 17 列（含夹爪；夹爪列会被解析但不回放）
 *
 * 用法（在 cpp/build/ 目录下）：
 *   ./teach_replay
 *   ./teach_replay ../examples/teach_points_left_right.txt
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
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

constexpr const char* kDefaultTeachFile = "../examples/teach_points_left_right.txt";

constexpr float kMaxSpeedDegS = 50.0F;
constexpr float kPositionToleranceDeg = 0.5F;
constexpr int kReachTimeoutSeconds = 10;
constexpr float kDwellSeconds = 1.0F;

std::atomic<bool> g_stop_requested{false};

void handle_sigint(int) {
    g_stop_requested.store(true);
}

struct TeachPoint {
    std::string label;
    std::vector<float> left_rad;
    std::vector<float> right_rad;
};

std::string trim(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::vector<float> deg_list_to_rad(const std::vector<float>& degrees) {
    std::vector<float> radians(degrees.size());
    for (std::size_t i = 0; i < degrees.size(); ++i)
        radians[i] = degrees[i] * kDegToRad;
    return radians;
}

void print_joints_deg(const char* prefix, const std::vector<float>& joints_rad) {
    std::cout << prefix << ": [";
    for (std::size_t i = 0; i < joints_rad.size(); ++i) {
        std::cout << joints_rad[i] * kRadToDeg;
        if (i + 1 < joints_rad.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

std::vector<TeachPoint> load_teach_points(const std::string& filepath) {
    std::vector<TeachPoint> points;
    std::ifstream input(filepath);
    if (!input.is_open()) {
        std::cerr << "Failed to open teach file: " << filepath << '\n';
        return points;
    }

    std::string line;
    while (std::getline(input, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        std::vector<std::string> cols;
        std::stringstream stream(line);
        std::string cell;
        while (std::getline(stream, cell, ',')) cols.push_back(trim(cell));

        if (cols.size() != 17 && cols.size() != 15) {
            std::cout << "Skip malformed line (expected 15 or 17 columns, got "
                      << cols.size() << "): " << line << '\n';
            continue;
        }

        std::vector<float> left_deg;
        std::vector<float> right_deg;
        left_deg.reserve(7);
        right_deg.reserve(7);
        for (int i = 1; i <= 7; ++i) left_deg.push_back(std::stof(cols[i]));
        // 15 cols: idx + left7 + right7; 17 cols: idx + left7 + gripper + right7 + gripper
        const int right_start = cols.size() == 17 ? 9 : 8;
        for (int i = right_start; i < right_start + 7; ++i)
            right_deg.push_back(std::stof(cols[i]));

        points.push_back(TeachPoint{
            cols[0],
            deg_list_to_rad(left_deg),
            deg_list_to_rad(right_deg),
        });
    }
    return points;
}

bool wait_for_reach(ControlApi& api,
                    const std::vector<float>& left_target,
                    const std::vector<float>& right_target,
                    float tolerance_rad, float poll_interval_s,
                    int timeout_seconds, int settle_reads) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout_seconds);
    int stable = 0;
    auto last_progress = std::chrono::steady_clock::now();
    float last_error = 0.0F;

    while (std::chrono::steady_clock::now() <= deadline) {
        auto [left_code, left_current] = api.get_joint(Group::LEFT_ARM);
        auto [right_code, right_current] = api.get_joint(Group::RIGHT_ARM);
        if (left_code != RetCode::SUCCESS || right_code != RetCode::SUCCESS ||
            left_current.size() != left_target.size() ||
            right_current.size() != right_target.size()) {
            std::this_thread::sleep_for(std::chrono::duration<float>(poll_interval_s));
            continue;
        }

        float max_error = 0.0F;
        for (std::size_t i = 0; i < left_target.size(); ++i)
            max_error = std::max(max_error,
                                 std::abs(left_current[i] - left_target[i]));
        for (std::size_t i = 0; i < right_target.size(); ++i)
            max_error = std::max(max_error,
                                 std::abs(right_current[i] - right_target[i]));
        last_error = max_error;

        if (max_error <= tolerance_rad) {
            if (++stable >= settle_reads) {
                std::cout << "    reached target (max joint err "
                          << max_error * kRadToDeg << " deg)\n";
                return true;
            }
        } else {
            stable = 0;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_progress >= std::chrono::seconds(5)) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                                     now - (deadline - std::chrono::seconds(timeout_seconds)))
                                     .count();
            std::cout << "    moving... max joint err "
                      << max_error * kRadToDeg << " deg (" << elapsed << "s)\n";
            last_progress = now;
        }

        std::this_thread::sleep_for(std::chrono::duration<float>(poll_interval_s));
    }

    std::cout << "    WARNING: timed out after " << timeout_seconds
              << "s (last max joint err " << last_error * kRadToDeg
              << " deg)\n";
    return false;
}

bool move_both_arms(ControlApi& api, const std::vector<float>& left_target,
                    const std::vector<float>& right_target,
                    const std::vector<float>& head_hold) {
    std::vector<float> all;
    all.reserve(left_target.size() + head_hold.size() + right_target.size());
    all.insert(all.end(), left_target.begin(), left_target.end());
    all.insert(all.end(), head_hold.begin(), head_hold.end());
    all.insert(all.end(), right_target.begin(), right_target.end());

    const RetCode code = api.move_joint(Group::ALL, all, false);
    if (code != RetCode::SUCCESS) {
        std::cerr << "move_joint(ALL): " << ret_code_name(code) << '\n';
        return false;
    }
    return true;
}

bool configure_arm_speed(ControlApi& api, Group group, std::size_t joint_count) {
    const std::vector<float> requested_speed(joint_count, kMaxSpeedDegS * kDegToRad);
    const std::vector<float> applied =
        api.set_joint_max_speed(group, requested_speed);
    if (applied.size() != requested_speed.size()) {
        std::cerr << "set_joint_max_speed failed for group "
                  << static_cast<int>(group) << '\n';
        return false;
    }
    return true;
}

std::string default_teach_file_path() {
    return std::string(kDefaultTeachFile);
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const std::string teach_file =
            argc > 1 ? argv[1] : default_teach_file_path();

        ControlApi api(kConfigPath);
        std::vector<float> left_current;
        std::vector<float> head_current;

        std::cout << "Waiting for rt_control state...\n";
        if (!wait_for_joint_state(api, Group::LEFT_ARM, left_current, 3000)) {
            std::cerr << "No state received within 3 seconds. Is rt_control running, "
                         "and do the UDP addresses/ports match config.yaml?\n";
            return 1;
        }
        if (!wait_for_joint_state(api, Group::HEAD, head_current, 3000)) {
            std::cerr << "No head state received within 3 seconds.\n";
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS) {
            std::cerr << "Failed to enable both arms.\n";
            return 1;
        }
        std::cout << "Both arms enabled\n";

        if (!configure_arm_speed(api, Group::LEFT_ARM, left_current.size()) ||
            !configure_arm_speed(api, Group::RIGHT_ARM, left_current.size())) {
            return 1;
        }
        std::cout << "Joint max speed set to " << kMaxSpeedDegS << " deg/s\n";

        const std::vector<TeachPoint> points = load_teach_points(teach_file);
        std::cout << "Loaded " << points.size() << " point(s) from "
                  << teach_file << '\n';
        if (points.empty()) {
            std::cout << "No points to replay.\n";
            return 0;
        }

        const float tolerance_rad = kPositionToleranceDeg * kDegToRad;

        std::signal(SIGINT, handle_sigint);
        std::cout << "Loop replay started. Press Ctrl+C to stop.\n";

        int cycle = 0;
        while (!g_stop_requested.load()) {
            ++cycle;
            std::cout << "\n========== Cycle " << cycle << " ==========\n";

            for (std::size_t i = 0; i < points.size() && !g_stop_requested.load();
                 ++i) {
                const auto& point = points[i];
                std::cout << "\n==> Replay point #" << (i + 1)
                          << " (record idx " << point.label << "):\n";
                print_joints_deg("    left_j", point.left_rad);
                print_joints_deg("    right_j", point.right_rad);

                if (!move_both_arms(api, point.left_rad, point.right_rad,
                                    head_current))
                    return 1;
                wait_for_reach(api, point.left_rad, point.right_rad,
                               tolerance_rad, 0.2F, kReachTimeoutSeconds, 3);

                if (g_stop_requested.load()) break;

                std::cout << "    dwell " << static_cast<int>(kDwellSeconds)
                          << "s at point\n";
                const auto dwell_deadline = std::chrono::steady_clock::now() +
                                            std::chrono::duration<float>(
                                                kDwellSeconds);
                while (std::chrono::steady_clock::now() < dwell_deadline &&
                       !g_stop_requested.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }

        std::cout << "\nCtrl+C received, stopping motion...\n";
        std::cout << "Motion stopped\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal SDK error: " << error.what() << '\n';
        return 1;
    }
}
