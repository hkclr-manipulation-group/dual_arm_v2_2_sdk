/**
 * @file test_movel.cpp
 * @brief 关节空间姿态测试：MoveL 直线运动。
 *
 * 顺序：
 *   1. 左臂关节走到 103.512, -17.218, -29.020, 95.323, -1.659, 46.885, 8.550（度）
 *   2. 末端直线运动：位置增量 0.06, 0.06, 0.06（米，工具系 XYZ），姿态增量 50, -40, -60（度，工具系 XYZ）
 *   3. 再回到第 1 步的关节角
 *
 * 用法：
 *   ./bin/cpp/test_movel
 *   ./bin/cpp/test_movel /path/to/cuarm_configuration/dual_v2_2
 */

#include "control_api.h"
#include "example_common.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
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

constexpr uint16_t kSpeedRatio = 30;
constexpr float kMoveLineAccTimeS = 3.0F;
constexpr float kPosToleranceM = 0.001F;
constexpr float kRotToleranceRad = 0.01F * kDegToRad;
constexpr float kJointToleranceRad = 0.01F * kDegToRad;
constexpr int kMoveTimeoutMs = 40000;
constexpr int kInitialReachTimeoutMs = 40000;
constexpr int kPollMs = 50;
constexpr int kPoseStableCycles = 3;

const std::vector<float> kInitialDeg = {103.512F, -17.218F, -29.020F, 95.323F,
                                         -1.659F,  46.885F,  8.550F};

std::vector<float> deg_to_rad(const std::vector<float>& degrees) {
    std::vector<float> radians;
    radians.reserve(degrees.size());
    for (float degree : degrees) radians.push_back(degree * kDegToRad);
    return radians;
}

void print_joints_deg(const std::vector<float>& joints_rad) {
    std::cout << std::fixed;
    for (size_t i = 0; i < joints_rad.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout.precision(4);
        std::cout << joints_rad[i] * kRadToDeg;
    }
    std::cout << '\n';
}

void print_tool_pose(const char* label, const std::vector<float>& pose) {
    std::cout << label << '\n';
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  position (m): " << pose[0] << ", " << pose[1] << ", "
              << pose[2] << '\n';
    std::cout << "  quaternion:   " << pose[3] << ", " << pose[4] << ", "
              << pose[5] << ", " << pose[6] << '\n';
}

std::array<float, 4> euler_xyz_to_quat(float rx, float ry, float rz) {
    const float cx = std::cos(rx * 0.5F);
    const float sx = std::sin(rx * 0.5F);
    const float cy = std::cos(ry * 0.5F);
    const float sy = std::sin(ry * 0.5F);
    const float cz = std::cos(rz * 0.5F);
    const float sz = std::sin(rz * 0.5F);
    float w = cx * cy * cz - sx * sy * sz;
    float x = sx * cy * cz + cx * sy * sz;
    float y = cx * sy * cz - sx * cy * sz;
    float z = cx * cy * sz + sx * sy * cz;
    const float n = std::sqrt(w * w + x * x + y * y + z * z);
    if (n > 1.0e-8F) {
        w /= n;
        x /= n;
        y /= n;
        z /= n;
    }
    return {w, x, y, z};
}

std::array<float, 4> quat_mul(const std::array<float, 4>& a,
                              const std::array<float, 4>& b) {
    std::array<float, 4> q{
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1.0e-8F) {
        for (float& c : q) c /= n;
    }
    return q;
}

/** 与 grab_book 的 common::offset_pose 相同：位置加增量，姿态右乘 XYZ 欧拉增量。 */
std::vector<float> offset_pose(const std::vector<float>& pose,
                               const std::array<float, 3>& xyz_offset_m,
                               const std::array<float, 3>& euler_delta_rad) {
    std::vector<float> target = pose;
    target[0] += xyz_offset_m[0];
    target[1] += xyz_offset_m[1];
    target[2] += xyz_offset_m[2];
    const std::array<float, 4> q_cur{target[3], target[4], target[5], target[6]};
    const std::array<float, 4> q_delta =
        euler_xyz_to_quat(euler_delta_rad[0], euler_delta_rad[1], euler_delta_rad[2]);
    const std::array<float, 4> q_new = quat_mul(q_cur, q_delta);
    target[3] = q_new[0];
    target[4] = q_new[1];
    target[5] = q_new[2];
    target[6] = q_new[3];
    return target;
}

/** q_cur^{-1} * q_tar，与 grab_book 的 get_relative_quat 相同。 */
std::array<float, 4> relative_quat(const std::vector<float>& pose,
                                   const std::vector<float>& target) {
    const float cw = pose[3];
    const float cx = -pose[4];
    const float cy = -pose[5];
    const float cz = -pose[6];
    const float tw = target[3];
    const float tx = target[4];
    const float ty = target[5];
    const float tz = target[6];
    std::array<float, 4> q{
        cw * tw - cx * tx - cy * ty - cz * tz,
        cw * tx + cx * tw + cy * tz - cz * ty,
        cw * ty - cx * tz + cy * tw + cz * tx,
        cw * tz + cx * ty - cy * tx + cz * tw,
    };
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 1.0e-6F) {
        for (float& c : q) c /= n;
    }
    return q;
}

/** 与 grab_book 的 quaternion_to_eigen_euler_xyz 相同，返回 roll/pitch/yaw（弧度）。 */
std::array<float, 3> quat_to_euler_xyz(float w, float x, float y, float z) {
    const float r00 = 1.0F - 2.0F * (y * y + z * z);
    const float r01 = 2.0F * (x * y - w * z);
    const float r02 = 2.0F * (x * z + w * y);
    const float r12 = 2.0F * (y * z - w * x);
    const float r22 = 1.0F - 2.0F * (x * x + y * y);

    const float pitch = std::asin(std::max(-1.0F, std::min(1.0F, r02)));
    const float cy = std::cos(pitch);
    float roll = 0.0F;
    float yaw = 0.0F;
    if (cy > 1.0e-4F) {
        roll = std::atan2(-r12, r22);
        yaw = std::atan2(-r01, r00);
    } else {
        const float r10 = 2.0F * (x * y + w * z);
        const float r11 = 1.0F - 2.0F * (x * x + z * z);
        roll = r02 > 0.0F ? std::atan2(r10, r11) : -std::atan2(r10, r11);
    }
    return {roll, pitch, yaw};
}

void print_pose_error(float pos_err, float rot_err, const std::array<float, 3>& rpy) {
    std::cout << "pos_err: " << pos_err << ", rot_err: " << rot_err
              << " (roll: " << rpy[0] * kRadToDeg
              << ", pitch: " << rpy[1] * kRadToDeg
              << ", yaw: " << rpy[2] * kRadToDeg << ")\n";
}

bool wait_for_joint_target(ControlApi& api, const std::vector<float>& target_rad) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kInitialReachTimeoutMs);
    int stable = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, current] = api.get_joint(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS && current.size() == target_rad.size()) {
            float max_err = 0.0F;
            for (size_t i = 0; i < current.size(); ++i)
                max_err = std::max(max_err, std::abs(current[i] - target_rad[i]));
            if (max_err <= kJointToleranceRad) {
                if (++stable >= 3) return true;
            } else {
                stable = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

bool wait_for_tool_pose(ControlApi& api, const std::vector<float>& target) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kMoveTimeoutMs);
    int stable = 0;
    int cnt = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, pose] = api.get_tool_pose(Group::LEFT_ARM);
        if (code == RetCode::SUCCESS && pose.size() >= 7) {
            const float dx = pose[0] - target[0];
            const float dy = pose[1] - target[1];
            const float dz = pose[2] - target[2];
            const float pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
            const std::array<float, 4> q_rel = relative_quat(pose, target);
            const float abs_w = std::min(1.0F, std::abs(q_rel[0]));
            const float rot_err = 2.0F * std::acos(abs_w);
            const std::array<float, 3> rpy =
                quat_to_euler_xyz(q_rel[0], q_rel[1], q_rel[2], q_rel[3]);
            if (pos_err <= kPosToleranceM && rot_err <= kRotToleranceRad) {
                if (++stable >= kPoseStableCycles) {
                    print_pose_error(pos_err, rot_err, rpy);
                    return true;
                }
            } else {
                stable = 0;
            }
            if (cnt % 10 == 0) print_pose_error(pos_err, rot_err, rpy);
            ++cnt;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    return false;
}

bool move_to_initial(ControlApi& api, const std::vector<float>& initial_rad) {
    const RetCode code = api.move_joint(Group::LEFT_ARM, initial_rad, false);
    if (code != RetCode::SUCCESS) {
        std::cerr << "move_joint failed: " << ret_code_name(code) << '\n';
        return false;
    }
    if (!wait_for_joint_target(api, initial_rad)) {
        std::cout << "  WARNING: hold pose not reached within "
                  << kInitialReachTimeoutMs << " ms\n";
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    ControlApi api(config_path_from_args(argc, argv));

    std::vector<float> probe;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
        std::cerr << "No state received within 3 s. Is rt_control running?\n";
        return 1;
    }
    if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS) {
        std::cerr << "Failed to enable left arm\n";
        return 1;
    }

    const std::vector<float> initial_rad = deg_to_rad(kInitialDeg);

    std::cout << "---------------Move to initial point------------\n";
    if (!move_to_initial(api, initial_rad)) return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    auto [pose_code, current] = api.get_tool_pose(Group::LEFT_ARM);
    if (pose_code != RetCode::SUCCESS || current.size() < 7) {
        std::cout << "get_tool_pose failed for " << group_name(Group::LEFT_ARM)
                  << '\n';
        return 1;
    }
    print_tool_pose("Current tool pose:", current);

    std::cout << "---------------Test motion 0------------\n";
    std::vector<float> joints;
    if (!wait_for_joint_state(api, Group::LEFT_ARM, joints)) {
        std::cerr << "No state received within 3 s. Is rt_control running?\n";
        return 1;
    }
    print_joints_deg(joints);

    std::cout << "---------------Test motion 0 move to target point------------\n";
    const std::array<float, 3> xyz_offset_m = {0.06F, 0.06F, 0.06F};
    const std::array<float, 3> euler_delta_deg = {50.0F, -40.0F, -60.0F};
    const std::array<float, 3> euler_delta_rad = {
        euler_delta_deg[0] * kDegToRad,
        euler_delta_deg[1] * kDegToRad,
        euler_delta_deg[2] * kDegToRad,
    };
    const std::vector<float> target =
        offset_pose(current, xyz_offset_m, euler_delta_rad);
    print_tool_pose("Target tool pose:", target);

    const RetCode line_code =
        api.move_line(Group::LEFT_ARM, target, kSpeedRatio, kMoveLineAccTimeS);
    std::cout << "move_line(LEFT_ARM): " << ret_code_name(line_code) << '\n';
    if (line_code != RetCode::SUCCESS) return 1;
    if (!wait_for_tool_pose(api, target)) {
        std::cout << "Warning: ACK ok but ToolPose did not reach target within "
                  << kMoveTimeoutMs << " ms\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "---------------Test motion 0 move back to initial point------------\n";
    if (!move_to_initial(api, initial_rad)) return 1;
    return 0;
}
