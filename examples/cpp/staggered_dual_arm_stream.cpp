/**
 * @file staggered_dual_arm_stream.cpp
 * @brief 错峰双臂高跟随：左先动 → 右再动（重叠）→ 左先回零 → 右再回零。
 *
 * 与 examples/c/staggered_dual_arm_stream.c 对齐。
 * 控制：set_left/right_joint_target + start_dual_arm_streaming (SDK auto flush)
 *
 * 用法（build/bin/cpp/ 下）：
 *   ./staggered_dual_arm_stream
 *   ./staggered_dual_arm_stream --left 20 --right 20
 *   ./staggered_dual_arm_stream --left 10,20,30,40,50,60,70 --right 15
 *   ./staggered_dual_arm_stream --right-lag 2 --right-return-lag 2 --move-duration 4
 *   ./staggered_dual_arm_stream /path/to/cuarm_configuration/dual_v2_2
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::config_path_from_args;
using example::kDegToRad;
using example::kPi;
using example::wait_for_joint_state;

namespace {

constexpr int kControlHz = 100;
constexpr int kPeriodMs = 10;
constexpr float kDefaultMoveDurationS = 4.0F;
constexpr float kDefaultRightLagS = 2.0F;
constexpr float kDefaultHoldS = 1.0F;
constexpr float kDefaultRightReturnLagS = 2.0F;
constexpr float kDefaultOffsetDeg = 20.0F;
constexpr float kMaxSpeedDegS = 30.0F;
constexpr uint8_t kTrajectoryMode = 2;
constexpr uint16_t kRadio = 999;

struct ArmPlan {
    std::vector<float> start;
    std::vector<float> target;
    std::vector<float> cmd;
    std::vector<float> offset_deg;
};

struct ParsedArgs {
    bool has_left = false;
    bool has_right = false;
    std::vector<float> left_deg;
    std::vector<float> right_deg;
    float move_duration_s = kDefaultMoveDurationS;
    float right_lag_s = kDefaultRightLagS;
    float hold_s = kDefaultHoldS;
    float right_return_lag_s = kDefaultRightReturnLagS;
    bool show_help = false;
};

using Clock = std::chrono::steady_clock;

float smooth_step(float u) {
    if (u <= 0.0F) return 0.0F;
    if (u >= 1.0F) return 1.0F;
    return 0.5F - 0.5F * std::cos(kPi * u);
}

void lerp_pose(std::vector<float>& out, const std::vector<float>& a,
               const std::vector<float>& b, float u) {
    const float s = smooth_step(u);
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = a[i] + (b[i] - a[i]) * s;
    }
}

std::optional<std::vector<float>> parse_offset_spec(const std::string& text) {
    std::vector<float> values;
    std::stringstream ss(text);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) return std::nullopt;
        try {
            size_t pos = 0;
            const float v = std::stof(token, &pos);
            if (pos != token.size()) return std::nullopt;
            values.push_back(v);
        } catch (...) {
            return std::nullopt;
        }
    }
    if (values.empty()) return std::nullopt;
    return values;
}

std::optional<std::vector<float>> resolve_offsets(
    const std::vector<float>* user, size_t joint_count, const char* label) {
    std::vector<float> out(joint_count);
    if (user == nullptr || user->empty()) {
        std::fill(out.begin(), out.end(), kDefaultOffsetDeg);
        return out;
    }
    if (user->size() == 1) {
        std::fill(out.begin(), out.end(), (*user)[0]);
        return out;
    }
    if (user->size() != joint_count) {
        std::cerr << label << " offset count must be 1 or " << joint_count
                  << ", got " << user->size() << '\n';
        return std::nullopt;
    }
    for (size_t i = 0; i < joint_count; ++i) out[i] = (*user)[i];
    return out;
}

ArmPlan build_arm_plan(const std::vector<float>& start,
                       const std::vector<float>& offset_deg) {
    ArmPlan plan;
    plan.start = start;
    plan.cmd = start;
    plan.offset_deg = offset_deg;
    plan.target.resize(start.size());
    for (size_t i = 0; i < start.size(); ++i) {
        plan.target[i] = start[i] + offset_deg[i] * kDegToRad;
    }
    return plan;
}

void fill_outbound_cmd(ArmPlan& plan, double t, double out_start, double out_end) {
    if (t <= out_start) {
        plan.cmd = plan.start;
        return;
    }
    if (t >= out_end) {
        plan.cmd = plan.target;
        return;
    }
    const float u = static_cast<float>((t - out_start) / (out_end - out_start));
    lerp_pose(plan.cmd, plan.start, plan.target, u);
}

void fill_return_cmd(ArmPlan& plan, double t, double home_start, double home_end) {
    const std::vector<float> zero(plan.target.size(), 0.0F);
    if (t <= home_start) {
        plan.cmd = plan.target;
        return;
    }
    if (t >= home_end) {
        plan.cmd = zero;
        return;
    }
    const float u = static_cast<float>((t - home_start) / (home_end - home_start));
    lerp_pose(plan.cmd, plan.target, zero, u);
}

void fill_left_cmd(ArmPlan& plan, double t, double out_start, double out_end,
                   double home_start, double home_end) {
    if (t < home_start) {
        fill_outbound_cmd(plan, t, out_start, out_end);
    } else {
        fill_return_cmd(plan, t, home_start, home_end);
    }
}

void fill_right_cmd(ArmPlan& plan, double t, double out_start, double out_end,
                    double home_start, double home_end) {
    if (t < out_start) {
        plan.cmd = plan.start;
        return;
    }
    if (t < home_start) {
        fill_outbound_cmd(plan, t, out_start, out_end);
        return;
    }
    fill_return_cmd(plan, t, home_start, home_end);
}

bool write_streaming_targets(ControlApi& api, const ArmPlan& left,
                             const ArmPlan& right) {
    if (api.set_left_joint_target(left.cmd) != RetCode::SUCCESS) return false;
    if (api.set_right_joint_target(right.cmd) != RetCode::SUCCESS) return false;
    return true;
}

bool configure_speed(ControlApi& api, Group group, size_t joint_count) {
    const std::vector<float> speed(joint_count, kMaxSpeedDegS * kDegToRad);
    return api.set_joint_max_speed(group, speed).size() == joint_count;
}

void print_usage(const char* prog) {
    std::cout
        << "Usage: " << prog << " [config_prefix] [options]\n"
        << "  Staggered dual-arm high-follow via set_left/right + SDK streaming.\n"
        << "\n"
        << "  --left  <deg>| <d1,d2,...>   left offset from start (default +20/joint)\n"
        << "  --right <deg>| <d1,d2,...>   right offset (default +20/joint)\n"
        << "  --move-duration <sec>       each move segment duration (default "
        << kDefaultMoveDurationS << ")\n"
        << "  --right-lag <sec>           delay before right starts outbound (default "
        << kDefaultRightLagS << ")\n"
        << "  --hold <sec>                hold at target before return (default "
        << kDefaultHoldS << ")\n"
        << "  --right-return-lag <sec>    delay before right starts return (default "
        << kDefaultRightReturnLagS << ")\n"
        << "\n"
        << "Timeline:\n"
        << "  t=0           left outbound\n"
        << "  t=right-lag   right outbound (overlap while left still moving)\n"
        << "  both at target, hold\n"
        << "  left return to absolute zero\n"
        << "  +right-return-lag  right return to zero\n"
        << "\n"
        << "Examples:\n"
        << "  " << prog << " --left 20 --right 20\n"
        << "  " << prog << " --left 10,20,30,40,50,60,70 --right 15 --right-lag 1.5\n";
}

std::optional<ParsedArgs> parse_args(int argc, char** argv) {
    ParsedArgs args;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            args.show_help = true;
            return args;
        }
        if (arg == "--left" || arg == "-l") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            const auto parsed = parse_offset_spec(argv[++i]);
            if (!parsed) {
                std::cerr << "invalid --left offset list\n";
                return std::nullopt;
            }
            args.left_deg = *parsed;
            args.has_left = true;
            continue;
        }
        if (arg == "--right" || arg == "-r") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            const auto parsed = parse_offset_spec(argv[++i]);
            if (!parsed) {
                std::cerr << "invalid --right offset list\n";
                return std::nullopt;
            }
            args.right_deg = *parsed;
            args.has_right = true;
            continue;
        }
        if (arg == "--move-duration") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            args.move_duration_s = std::stof(argv[++i]);
            continue;
        }
        if (arg == "--right-lag") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            args.right_lag_s = std::stof(argv[++i]);
            continue;
        }
        if (arg == "--hold") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            args.hold_s = std::stof(argv[++i]);
            continue;
        }
        if (arg == "--right-return-lag") {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a value\n";
                return std::nullopt;
            }
            args.right_return_lag_s = std::stof(argv[++i]);
            continue;
        }
        if (!arg.empty() && arg[0] == '-') {
            std::cerr << "unknown option '" << arg << "'\n";
            return std::nullopt;
        }
    }
    return args;
}

}  // namespace

int main(int argc, char* argv[]) {
    const auto args_opt = parse_args(argc, argv);
    if (!args_opt) return 1;
    if (args_opt->show_help) {
        print_usage(argv[0]);
        return 0;
    }
    const ParsedArgs args = *args_opt;

    ControlApi api(config_path_from_args(argc, argv));

    std::vector<float> left_start;
    std::vector<float> right_start;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, left_start) ||
        !wait_for_joint_state(api, Group::RIGHT_ARM, right_start)) {
        std::cerr << "No rt_control state within 3 seconds.\n";
        return 1;
    }

    if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
        !configure_speed(api, Group::LEFT_ARM, left_start.size()) ||
        !configure_speed(api, Group::RIGHT_ARM, right_start.size())) {
        std::cerr << "Failed to enable arms or set speed.\n";
        return 1;
    }

    const auto left_offsets = resolve_offsets(
        args.has_left ? &args.left_deg : nullptr, left_start.size(), "left");
    const auto right_offsets = resolve_offsets(
        args.has_right ? &args.right_deg : nullptr, right_start.size(), "right");
    if (!left_offsets || !right_offsets) return 1;

    ArmPlan left = build_arm_plan(left_start, *left_offsets);
    ArmPlan right = build_arm_plan(right_start, *right_offsets);

    const double move_d = static_cast<double>(args.move_duration_s);
    const double right_lag = static_cast<double>(args.right_lag_s);
    const double hold = static_cast<double>(args.hold_s);
    const double right_return_lag = static_cast<double>(args.right_return_lag_s);

    const double t_left_out_start = 0.0;
    const double t_right_out_start = right_lag;
    const double t_left_out_end = t_left_out_start + move_d;
    const double t_right_out_end = t_right_out_start + move_d;
    const double t_return_start =
        std::max(t_left_out_end, t_right_out_end) + hold;
    const double t_left_home_start = t_return_start;
    const double t_right_home_start = t_return_start + right_return_lag;
    const double t_left_home_end = t_left_home_start + move_d;
    const double t_right_home_end = t_right_home_start + move_d;
    const double t_total = std::max(t_left_home_end, t_right_home_end) + 0.5;

    const double overlap_out_start = t_right_out_start;
    const double overlap_out_end = std::min(t_left_out_end, t_right_out_end);
    const double overlap_ret_start = t_right_home_start;
    const double overlap_ret_end = std::min(t_left_home_end, t_right_home_end);

    std::cout << "Config: " << config_path_from_args(argc, argv) << '\n';
    std::cout << "High-follow streaming @ " << kControlHz
              << " Hz, trajectory_mode=" << static_cast<unsigned>(kTrajectoryMode)
              << " radio=" << kRadio << '\n';
    std::cout << "Timeline (s):\n";
    std::cout << "  left  outbound: [" << t_left_out_start << ", "
              << t_left_out_end << "]\n";
    std::cout << "  right outbound: [" << t_right_out_start << ", "
              << t_right_out_end << "]\n";
    if (overlap_out_end > overlap_out_start) {
        std::cout << "  overlap outbound: [" << overlap_out_start << ", "
                  << overlap_out_end << "]\n";
    }
    std::cout << "  left  return:    [" << t_left_home_start << ", "
              << t_left_home_end << "]\n";
    std::cout << "  right return:    [" << t_right_home_start << ", "
              << t_right_home_end << "]\n";
    if (overlap_ret_end > overlap_ret_start) {
        std::cout << "  overlap return:  [" << overlap_ret_start << ", "
                  << overlap_ret_end << "]\n";
    }
    std::cout << "  total duration:  " << t_total << " s\n\n";

    if (api.start_dual_arm_streaming(true, kTrajectoryMode, kRadio, kControlHz) !=
        RetCode::SUCCESS) {
        std::cerr << "start_dual_arm_streaming failed\n";
        return 1;
    }

    const auto t0 = Clock::now();
    auto next = t0;

    while (true) {
        const double t = std::chrono::duration<double>(Clock::now() - t0).count();
        if (t >= t_total) break;

        fill_left_cmd(left, t, t_left_out_start, t_left_out_end,
                      t_left_home_start, t_left_home_end);
        fill_right_cmd(right, t, t_right_out_start, t_right_out_end,
                       t_right_home_start, t_right_home_end);

        if (!write_streaming_targets(api, left, right)) {
            std::cerr << "streaming write failed at t=" << t << '\n';
            break;
        }

        next += std::chrono::milliseconds(kPeriodMs);
        std::this_thread::sleep_until(next);
    }

    left.cmd.assign(left.start.size(), 0.0F);
    right.cmd.assign(right.start.size(), 0.0F);
    write_streaming_targets(api, left, right);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    api.stop_dual_arm_streaming();
    std::cout << "Done.\n";
    return 0;
}
