/**
 * @file test_get_joint_all.cpp
 * @brief 测试 get_joint(Group::ALL)：读取并校验左臂 + 头部 + 右臂拼接结果。
 *
 * Group::ALL 关节顺序：LEFT_ARM (J1..J7) + HEAD (J1..J2) + RIGHT_ARM (J1..J7)
 *
 * 前置条件：rt_control 已启动。
 *
 * 用法（在 build/bin/ 目录下）：
 *   ./test_get_joint_all
 *   ./test_get_joint_all --watch          # 每 500 ms 刷新
 *   ./test_get_joint_all --watch 10       # 每 10 ms 刷新（仅显示 ALL，不做逐次一致性校验）
 *   ./test_get_joint_all --strict         # 静止时严格校验（容差 0.001 deg）
 *   ./test_get_joint_all /path/to/cuarm_configuration/dual_v2_2
 *
 * 注意：每次 get_joint() 独立取 UDP 状态快照，连续多次调用并非同一时刻。
 *       高频 --watch 下请依赖 get_joint(ALL) 的单次结果；一致性校验仅在非 watch
 *       模式或 --strict 静止测试时使用。
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
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
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

struct SegmentLayout {
    std::size_t left{0};
    std::size_t head{0};
    std::size_t right{0};

    std::size_t total() const { return left + head + right; }
};

struct Options {
    std::string config_path;
    bool watch{false};
    bool strict{false};
    int watch_interval_ms{500};
};

void print_usage(const char* prog) {
    std::cerr
        << "Usage:\n"
        << "  " << prog << " [config_prefix] [--watch [interval_ms]] [--strict]\n\n"
        << "Examples:\n"
        << "  " << prog << '\n'
        << "  " << prog << " --watch\n"
        << "  " << prog << " --watch 10\n"
        << "  " << prog << " --strict\n";
}

bool parse_options(int argc, char* argv[], Options* opts) {
    opts->config_path = config_path_from_args(argc, argv);

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        }
        if (arg == "--watch") {
            opts->watch = true;
            if (i + 1 < argc) {
                const char* next = argv[i + 1];
                if (next[0] != '-') {
                    opts->watch_interval_ms = std::atoi(next);
                    if (opts->watch_interval_ms <= 0) opts->watch_interval_ms = 500;
                    ++i;
                }
            }
            continue;
        }
        if (arg == "--strict") {
            opts->strict = true;
            continue;
        }
        if (arg.rfind("--", 0) == 0) {
            std::cerr << "Unknown option: " << arg << '\n';
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

void print_joints_deg(const char* label, const std::vector<float>& joints_rad) {
    std::cout << std::fixed << std::setprecision(3) << label << " (" << joints_rad.size()
              << " joints, deg): [";
    for (std::size_t i = 0; i < joints_rad.size(); ++i) {
        std::cout << joints_rad[i] * kRadToDeg;
        if (i + 1 < joints_rad.size()) std::cout << ", ";
    }
    std::cout << "]\n";
}

void print_segment(const char* segment_label, std::size_t offset,
                   std::size_t count, const std::vector<float>& all_rad) {
    std::cout << "  " << segment_label << ": ";
    if (count == 0 || offset >= all_rad.size()) {
        std::cout << "(empty)\n";
        return;
    }
    const std::size_t end =
        std::min(offset + count, all_rad.size());
    std::cout << std::fixed << std::setprecision(3);
    for (std::size_t i = offset; i < end; ++i) {
        std::cout << "J" << i - offset + 1 << "=" << all_rad[i] * kRadToDeg << " deg";
        if (i + 1 < end) std::cout << ", ";
    }
    std::cout << '\n';
}

std::vector<float> concat_groups(const std::vector<float>& left,
                                 const std::vector<float>& head,
                                 const std::vector<float>& right) {
    std::vector<float> merged;
    merged.reserve(left.size() + head.size() + right.size());
    merged.insert(merged.end(), left.begin(), left.end());
    merged.insert(merged.end(), head.begin(), head.end());
    merged.insert(merged.end(), right.begin(), right.end());
    return merged;
}

float max_abs_diff(const std::vector<float>& a, const std::vector<float>& b) {
    const std::size_t n = std::min(a.size(), b.size());
    float max_diff = 0.0F;
    for (std::size_t i = 0; i < n; ++i)
        max_diff = std::max(max_diff, std::abs(a[i] - b[i]));
    return max_diff;
}

float max_abs_value(const std::vector<float>& values) {
    float max_value = 0.0F;
    for (float value : values)
        max_value = std::max(max_value, std::abs(value));
    return max_value;
}

bool probe_segment_layout(ControlApi& api, SegmentLayout* layout) {
    auto [left_code, left_joints] = api.get_joint(Group::LEFT_ARM);
    auto [head_code, head_joints] = api.get_joint(Group::HEAD);
    auto [right_code, right_joints] = api.get_joint(Group::RIGHT_ARM);
    if (left_code != RetCode::SUCCESS || head_code != RetCode::SUCCESS ||
        right_code != RetCode::SUCCESS) {
        return false;
    }
    layout->left = left_joints.size();
    layout->head = head_joints.size();
    layout->right = right_joints.size();
    return true;
}

bool read_watch(ControlApi& api, const SegmentLayout& layout) {
    auto [all_code, all_joints] = api.get_joint(Group::ALL);
    auto [vel_code, all_vel] = api.get_joint_velocity(Group::ALL);

    std::cout << "\n--- get_joint(ALL) ---\n";
    std::cout << "  status: " << ret_code_name(all_code)
              << "  size=" << all_joints.size() << '\n';
    if (all_code != RetCode::SUCCESS) {
        std::cerr << "get_joint(ALL) failed.\n";
        return false;
    }
    if (all_joints.size() != layout.total()) {
        std::cerr << "Unexpected ALL size: got " << all_joints.size()
                  << ", expected " << layout.total() << '\n';
        return false;
    }

    print_joints_deg("ALL", all_joints);
    if (vel_code == RetCode::SUCCESS && all_vel.size() == all_joints.size()) {
        print_joints_deg("ALL velocity", all_vel);
    }

    std::cout << "\nALL segments (deg):\n";
    print_segment("LEFT_ARM", 0, layout.left, all_joints);
    print_segment("HEAD", layout.left, layout.head, all_joints);
    print_segment("RIGHT_ARM", layout.left + layout.head, layout.right, all_joints);
    return true;
}

bool read_once(ControlApi& api, bool strict) {
    const auto t_start = std::chrono::steady_clock::now();

    auto [left_code, left_joints] = api.get_joint(Group::LEFT_ARM);
    auto [head_code, head_joints] = api.get_joint(Group::HEAD);
    auto [right_code, right_joints] = api.get_joint(Group::RIGHT_ARM);
    auto [all_code, all_joints] = api.get_joint(Group::ALL);

    const auto t_end = std::chrono::steady_clock::now();
    const float elapsed_s =
        std::chrono::duration<float>(t_end - t_start).count();

    auto [vel_code, all_vel] = api.get_joint_velocity(Group::ALL);

    std::cout << "\n--- get_joint snapshot ---\n";
    std::cout << "  get_joint(ALL):       " << ret_code_name(all_code)
              << "  size=" << all_joints.size() << '\n';
    std::cout << "  get_joint(LEFT_ARM):  " << ret_code_name(left_code)
              << "  size=" << left_joints.size() << '\n';
    std::cout << "  get_joint(HEAD):      " << ret_code_name(head_code)
              << "  size=" << head_joints.size() << '\n';
    std::cout << "  get_joint(RIGHT_ARM): " << ret_code_name(right_code)
              << "  size=" << right_joints.size() << '\n';
    std::cout << std::fixed << std::setprecision(3)
              << "  read span: " << elapsed_s * 1000.0F << " ms "
              << "(each call is an independent UDP snapshot)\n";

    if (all_code != RetCode::SUCCESS || left_code != RetCode::SUCCESS ||
        head_code != RetCode::SUCCESS || right_code != RetCode::SUCCESS) {
        std::cerr << "One or more get_joint calls failed.\n";
        return false;
    }

    const std::vector<float> expected =
        concat_groups(left_joints, head_joints, right_joints);
    const std::size_t expected_size =
        left_joints.size() + head_joints.size() + right_joints.size();

    std::cout << "\nExpected ALL layout: LEFT(" << left_joints.size()
              << ") + HEAD(" << head_joints.size() << ") + RIGHT("
              << right_joints.size() << ") = " << expected_size << " joints\n";

    print_joints_deg("ALL", all_joints);
    print_joints_deg("LEFT_ARM", left_joints);
    print_joints_deg("HEAD", head_joints);
    print_joints_deg("RIGHT_ARM", right_joints);

    if (all_joints.size() != expected_size) {
        std::cerr << "FAIL: ALL size mismatch. got " << all_joints.size()
                  << ", expected " << expected_size << '\n';
        return false;
    }

    const float diff_rad = max_abs_diff(all_joints, expected);
    const float diff_deg = diff_rad * kRadToDeg;

    float max_vel_rad_s = 0.0F;
    if (vel_code == RetCode::SUCCESS) max_vel_rad_s = max_abs_value(all_vel);

    constexpr float kStrictToleranceDeg = 1e-3F;
    constexpr float kDefaultFloorToleranceDeg = 0.05F;
    const float motion_tolerance_deg = max_vel_rad_s * kRadToDeg * elapsed_s * 2.0F;
    const float tolerance_deg =
        strict ? kStrictToleranceDeg
               : std::max(kDefaultFloorToleranceDeg, motion_tolerance_deg);

    std::cout << std::fixed << std::setprecision(6)
              << "Consistency check: max |ALL - concat(L,H,R)| = "
              << diff_rad << " rad (" << diff_deg << " deg), "
              << "tolerance " << tolerance_deg << " deg\n";

    if (diff_deg > tolerance_deg) {
        std::cerr << "FAIL: ALL joints differ from concatenated groups "
                  << "(likely UDP snapshot drift across sequential calls).\n";
        return false;
    }

    std::cout << "PASS: get_joint(ALL) matches LEFT + HEAD + RIGHT.\n";

    std::cout << "\nALL segments (deg):\n";
    print_segment("LEFT_ARM", 0, left_joints.size(), all_joints);
    print_segment("HEAD", left_joints.size(), head_joints.size(), all_joints);
    print_segment("RIGHT_ARM", left_joints.size() + head_joints.size(),
                  right_joints.size(), all_joints);

    if (vel_code == RetCode::SUCCESS && all_vel.size() == all_joints.size()) {
        print_joints_deg("ALL velocity", all_vel);
    } else {
        std::cout << "get_joint_velocity(ALL): " << ret_code_name(vel_code)
                  << "  size=" << all_vel.size() << '\n';
    }

    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    Options opts;
    if (!parse_options(argc, argv, &opts)) {
        return argc > 1 ? 1 : 0;
    }

    ControlApi api(opts.config_path);
    std::cout << "Config: " << opts.config_path << '\n'
              << "Testing " << group_name(Group::ALL)
              << " via get_joint()\n";

    std::vector<float> probe;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
        std::cerr << "No state received within 3 s. Is rt_control running?\n";
        return 1;
    }

    SegmentLayout layout;
    if (!probe_segment_layout(api, &layout)) {
        std::cerr << "Failed to probe segment layout.\n";
        return 1;
    }

    if (!opts.watch) {
        return read_once(api, opts.strict) ? 0 : 1;
    }

    std::cout << "Watch mode: refresh every " << opts.watch_interval_ms
              << " ms, only get_joint(ALL) per tick (Ctrl+C to stop)\n"
              << "Layout: LEFT(" << layout.left << ") + HEAD(" << layout.head
              << ") + RIGHT(" << layout.right << ")\n";

    int iteration = 0;
    while (true) {
        std::cout << "\n========== iteration " << ++iteration << " ==========\n";
        if (!read_watch(api, layout)) return 1;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(opts.watch_interval_ms));
    }
}
