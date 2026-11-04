/**
 * @file dual_arm_async_joint_cartesian.cpp
 * @brief 左 Joint + 右 Tool 直线（SDK IK）异步低跟随；双臂同包 joint streaming。
 *
 * Joint 臂：相对当前反馈的 MoveJ（Δ关节角，度）。
 * Task 臂：Tool 直线（SDK IK）。打印起/目标/终止（关节 + Tool）及双臂误差。
 *
 * 用法：
 *   ./dual_arm_async_joint_cartesian [config_prefix]
 *   ./dual_arm_async_joint_cartesian --cartesian left
 *   ./dual_arm_async_joint_cartesian --joint-duration 4
 */

#include "async_dual_arm_controller.h"
#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::AsyncDualArmController;
using dual_arm_v2_2_sdk::AsyncMotionState;
using dual_arm_v2_2_sdk::AsyncStreamOptions;
using dual_arm_v2_2_sdk::CartesianSide;
using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::Joint7Rad;
using dual_arm_v2_2_sdk::MotionRetCode;
using dual_arm_v2_2_sdk::RetCode;
using dual_arm_v2_2_sdk::ToolLineSpec;
using dual_arm_v2_2_sdk::ToolPose7;
using example::config_path_from_args;
using example::group_name;
using example::kDegToRad;
using example::kRadToDeg;
using example::wait_for_joint_state;

namespace {

/** Joint 臂相对运动：各关节偏移（度），相对 start 时刻反馈。 */
constexpr std::initializer_list<float> kJointRelativeDeltaDeg = {-15.0F, 10.0F, -10.0F,
                                                                 10.0F, 10.0F, 10.0F,
                                                                 0.0F};

constexpr float kTaskLineDeltaZM = 0.10F;
constexpr float kDefaultJointMoveDurationS = 4.0F;

Joint7Rad joints_deg(std::initializer_list<float> deg) {
    Joint7Rad q{};
    size_t i = 0;
    for (float d : deg) {
        if (i < q.size()) q[i++] = d * kDegToRad;
    }
    return q;
}

Group group_for_side(CartesianSide side) {
    return side == CartesianSide::LEFT ? Group::LEFT_ARM : Group::RIGHT_ARM;
}

const char* side_label(CartesianSide side) {
    return side == CartesianSide::LEFT ? "LEFT" : "RIGHT";
}

const char* motion_ret_name(MotionRetCode c) {
    switch (c) {
        case MotionRetCode::SUCCESS: return "SUCCESS";
        case MotionRetCode::IK_FAILED: return "IK_FAILED";
        case MotionRetCode::IK_NEAR_SINGULAR: return "IK_NEAR_SINGULAR";
        case MotionRetCode::PATH_BLOCKED: return "PATH_BLOCKED";
        case MotionRetCode::NO_STATE: return "NO_STATE";
        default: return "OTHER";
    }
}

bool read_joint(ControlApi& api, Group g, Joint7Rad& out) {
    const auto [code, j] = api.get_joint(g);
    if (code != RetCode::SUCCESS || j.size() != 7) return false;
    for (int i = 0; i < 7; ++i) out[i] = j[i];
    return true;
}

bool read_tool(ControlApi& api, Group g, ToolPose7& out) {
    const auto [code, p] = api.get_tool_pose(g);
    if (code != RetCode::SUCCESS || p.size() != 7) return false;
    for (int i = 0; i < 7; ++i) out[i] = p[i];
    return true;
}

void print_joint_deg(const char* label, const Joint7Rad& q) {
    std::cout << "  " << label << " joint(deg):";
    for (float v : q) {
        std::cout << ' ' << std::fixed << std::setprecision(3) << (v * kRadToDeg);
    }
    std::cout << '\n';
}

void print_tool_pose(const char* label, const ToolPose7& p) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  " << label << " tool: pos(m)=(" << p[0] << ", " << p[1] << ", "
              << p[2] << ") quat(wxyz)=(" << p[3] << ", " << p[4] << ", " << p[5]
              << ", " << p[6] << ")\n";
}

void print_arm_snapshot(ControlApi& api, Group g, const char* phase) {
    std::cout << "[" << phase << " " << group_name(g) << "]\n";
    Joint7Rad q{};
    ToolPose7 t{};
    if (read_joint(api, g, q)) print_joint_deg("feedback", q);
    else std::cout << "  feedback joint: N/A\n";
    if (read_tool(api, g, t)) print_tool_pose("feedback", t);
    else std::cout << "  feedback tool: N/A\n";
}

float tool_pos_error_m(const ToolPose7& a, const ToolPose7& b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float tool_ori_error_rad(const ToolPose7& a, const ToolPose7& b) {
    const float dot = std::abs(a[3] * b[3] + a[4] * b[4] + a[5] * b[5] +
                               a[6] * b[6]);
    return 2.0F * std::acos(std::min(1.0F, dot));
}

float max_joint_error_deg(const Joint7Rad& a, const Joint7Rad& b, float& sum_sq) {
    sum_sq = 0.0F;
    float m = 0.0F;
    for (int i = 0; i < 7; ++i) {
        const float d = (a[i] - b[i]) * kRadToDeg;
        sum_sq += d * d;
        m = std::max(m, std::abs(d));
    }
    return m;
}

bool wait_task_reach(AsyncDualArmController& ctrl, float tol_m, float tol_ori_rad,
                     int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        float pe = 0.0F;
        float oe = 0.0F;
        if (ctrl.poll_reach(&pe, &oe) == MotionRetCode::SUCCESS) {
            std::cout << "Task arm in tolerance: pos " << pe * 1000.0F << " mm, ori "
                      << oe << " rad\n";
            return true;
        }
        if (ctrl.state() == AsyncMotionState::ERROR) {
            std::cerr << "Task arm error: " << motion_ret_name(ctrl.last_error())
                      << '\n';
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

CartesianSide parse_cartesian_side(int argc, char* argv[]) {
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--cartesian" || a == "-c") {
            const std::string v = argv[i + 1];
            if (v == "left" || v == "L") return CartesianSide::LEFT;
            if (v == "right" || v == "R") return CartesianSide::RIGHT;
        }
    }
    return CartesianSide::RIGHT;
}

float parse_joint_duration_s(int argc, char* argv[]) {
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--joint-duration" || a == "-jd") {
            return std::max(0.1F, std::stof(argv[i + 1]));
        }
    }
    return kDefaultJointMoveDurationS;
}

}  // namespace

int main(int argc, char* argv[]) {
#ifndef DUAL_ARM_V2_2_SDK_HAS_KINEMATICS
    std::cerr << "This example requires SDK built with Kinematics "
                 "(DUAL_ARM_V2_2_SDK_ENABLE_KINEMATICS=ON).\n";
    return 2;
#else
    const CartesianSide cartesian = parse_cartesian_side(argc, argv);
    const CartesianSide joint_side =
        cartesian == CartesianSide::LEFT ? CartesianSide::RIGHT
                                         : CartesianSide::LEFT;

    const Group task_group = group_for_side(cartesian);
    const Group joint_group = group_for_side(joint_side);

    const std::string config_prefix = config_path_from_args(argc, argv);
    const float joint_move_duration_s = parse_joint_duration_s(argc, argv);

    std::cout << "Roles: Cartesian=" << side_label(cartesian)
              << " Joint=" << side_label(joint_side)
              << " joint_move_duration=" << joint_move_duration_s << " s\n";

    try {
        ControlApi api(config_prefix);
        std::vector<float> probe;
        if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
            std::cerr << "No rt_control state within 3 s.\n";
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::HEAD, false) != RetCode::SUCCESS) {
            std::cerr << "Enable L/R failed.\n";
            return 1;
        }

        std::cout << "\n=== START (both arms) ===\n";
        print_arm_snapshot(api, Group::LEFT_ARM, "START");
        print_arm_snapshot(api, Group::RIGHT_ARM, "START");

        AsyncStreamOptions opt{};
        opt.coalesce_hz = 100;
        opt.cartesian_arm = cartesian;
        opt.tool_pos_tolerance_m = 0.0005F;
        opt.tool_ori_tolerance_rad = 0.00872665F;
        opt.singularity.reject_entire_line = true;
        opt.singularity.sigma_min_threshold = 0.02F;

        AsyncDualArmController ctrl(api, config_prefix);
        if (ctrl.start(opt) != MotionRetCode::SUCCESS) {
            std::cerr << "AsyncDualArmController::start failed.\n";
            return 1;
        }

        Joint7Rad q_joint_start{};
        if (!read_joint(api, joint_group, q_joint_start)) {
            std::cerr << "read joint arm start failed.\n";
            ctrl.stop();
            return 1;
        }

        const Joint7Rad delta_rad = joints_deg(kJointRelativeDeltaDeg);
        Joint7Rad q_joint_target{};
        for (int i = 0; i < 7; ++i) {
            q_joint_target[i] = q_joint_start[i] + delta_rad[i];
        }

        std::cout << "\n=== JOINT ARM TARGET (relative MoveJ) ===\n";
        print_joint_deg("start", q_joint_start);
        print_joint_deg("delta", delta_rad);
        print_joint_deg("target", q_joint_target);
        ToolPose7 tool_joint_start{};
        if (read_tool(api, joint_group, tool_joint_start)) {
            print_tool_pose("start tool (Cartesian)", tool_joint_start);
        }

        std::cout << "Submit joint set_joint_target + Task line...\n";
        if (ctrl.set_joint_target(joint_side, q_joint_target, joint_move_duration_s) !=
            MotionRetCode::SUCCESS) {
            std::cerr << "set_joint_target failed.\n";
            ctrl.stop();
            return 1;
        }

        ToolPose7 tool_task_start{};
        if (!read_tool(api, task_group, tool_task_start)) {
            std::cerr << "get_tool_pose (task start) failed.\n";
            ctrl.stop();
            return 1;
        }

        ToolPose7 tool_task_end = tool_task_start;
        tool_task_end[2] += kTaskLineDeltaZM;

        std::cout << "\n=== TASK ARM TARGET (Tool line) ===\n";
        print_tool_pose("start", tool_task_start);
        print_tool_pose("end", tool_task_end);

        ToolLineSpec line{};
        line.end = tool_task_end;
        line.start_is_current = true;
        line.speed_ratio = 12;
        line.acc_time_s = 4.0F;

        const MotionRetCode line_code = ctrl.move_tool_line(line);
        if (line_code != MotionRetCode::SUCCESS) {
            std::cerr << "move_tool_line: " << motion_ret_name(line_code) << '\n';
            ctrl.stop();
            return 1;
        }

        if (!wait_task_reach(ctrl, opt.tool_pos_tolerance_m,
                             opt.tool_ori_tolerance_rad, 120000)) {
            std::cerr << "Task arm did not reach within tolerance.\n";
        }

        std::cout << "\n=== END (both arms) ===\n";
        print_arm_snapshot(api, Group::LEFT_ARM, "END");
        print_arm_snapshot(api, Group::RIGHT_ARM, "END");

        Joint7Rad q_joint_end{};
        ToolPose7 tool_task_end_fb{};
        ToolPose7 tool_joint_end{};
        read_joint(api, joint_group, q_joint_end);
        read_tool(api, task_group, tool_task_end_fb);
        read_tool(api, joint_group, tool_joint_end);

        std::cout << "\n=== ERROR SUMMARY ===\n";
        float joint_rms = 0.0F;
        const float joint_max_deg =
            max_joint_error_deg(q_joint_end, q_joint_target, joint_rms);
        joint_rms = std::sqrt(joint_rms / 7.0F);
        std::cout << "Joint arm (" << side_label(joint_side)
                  << "): max |Δq| vs target = " << std::fixed
                  << std::setprecision(4) << joint_max_deg << " deg, RMS = "
                  << joint_rms << " deg\n";

        const float task_pos_mm =
            tool_pos_error_m(tool_task_end_fb, tool_task_end) * 1000.0F;
        const float task_ori_rad =
            tool_ori_error_rad(tool_task_end_fb, tool_task_end);
        std::cout << "Task arm (" << side_label(cartesian)
                  << "): Tool vs line end: pos " << task_pos_mm
                  << " mm, ori " << task_ori_rad << " rad\n";
        std::cout << "  (tolerance pos " << opt.tool_pos_tolerance_m * 1000.0F
                  << " mm, ori " << opt.tool_ori_tolerance_rad << " rad)\n";

        const bool task_ok =
            task_pos_mm <= opt.tool_pos_tolerance_m * 1000.0F + 1e-3F &&
            task_ori_rad <= opt.tool_ori_tolerance_rad + 1e-5F;

        std::cout << "\nDone. joint_move progress="
                  << ctrl.joint_move_progress(joint_side)
                  << " task progress=" << ctrl.cartesian_progress()
                  << " task_ok=" << (task_ok ? "yes" : "no") << '\n';
        ctrl.stop(true);
        return task_ok ? 0 : 1;
    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << '\n';
        return 1;
    }
#endif
}
