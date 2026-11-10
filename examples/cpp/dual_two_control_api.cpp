/**
 * @file dual_two_control_api.cpp
 * @brief 两个 rt_control：各一个 ControlApi，并行测试 MoveJ / MoveL / 静止的任意组合。
 *
 * 左 config 只命令 Group::LEFT_ARM（arm 0）；右 config 只命令 Group::RIGHT_ARM（arm 2）。
 * 默认关节角和工具系增量与 examples/cpp/test_movel.cpp 相同。
 *
 * 用法：
 *   ./dual_two_control_api <config_left> <config_right>
 *       [--left  movej|movel|hold] [--right movej|movel|hold]
 *       [--joints-left  j1,j2,j3,j4,j5,j6,j7]
 *       [--joints-right j1,j2,j3,j4,j5,j6,j7]
 *       [--dpos-left dx,dy,dz]  [--drpy-left  rx,ry,rz]
 *       [--dpos-right dx,dy,dz] [--drpy-right rx,ry,rz]
 *       [--speed-joint 0..100] [--speed-line 0..100] [--acc-line 秒]
 *
 * 关节角为绝对角度（度）。dpos 为工具系位置增量（米），drpy 为工具系 XYZ 欧拉增量（度）。
 * 不写 --left / --right 时两边都做 movej。静止侧只使能并保持当前关节，不发新运动。
 */

#include "control_api.h"
#include "example_common.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::group_name;
using example::kDegToRad;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr int kStateTimeoutMs = 5000;

// 与 test_movel.cpp 相同。
constexpr float kDefaultJointsDeg[7] = {103.512F, -17.218F, -29.020F, 95.323F,
                                         -1.659F,  46.885F,  8.550F};
constexpr float kDefaultDposM[3] = {0.06F, 0.06F, 0.06F};
constexpr float kDefaultDrpyDeg[3] = {50.0F, -40.0F, -60.0F};
constexpr uint16_t kDefaultJointSpeed = 30;
constexpr uint16_t kDefaultLineSpeed = 30;
constexpr float kDefaultLineAccS = 3.0F;

enum class MotionKind { Hold, MoveJ, MoveL };

struct ArmJob {
    std::string label;
    std::string config_prefix;
    Group group = Group::LEFT_ARM;
    MotionKind motion = MotionKind::MoveJ;
    std::array<float, 7> joints_deg = {kDefaultJointsDeg[0], kDefaultJointsDeg[1],
                                        kDefaultJointsDeg[2], kDefaultJointsDeg[3],
                                        kDefaultJointsDeg[4], kDefaultJointsDeg[5],
                                        kDefaultJointsDeg[6]};
    std::array<float, 3> dpos_m = {kDefaultDposM[0], kDefaultDposM[1], kDefaultDposM[2]};
    std::array<float, 3> drpy_deg = {kDefaultDrpyDeg[0], kDefaultDrpyDeg[1],
                                      kDefaultDrpyDeg[2]};
};

struct MotionLimits {
    uint16_t joint_speed = kDefaultJointSpeed;
    uint16_t line_speed = kDefaultLineSpeed;
    float line_acc_s = kDefaultLineAccS;
};

struct StartGate {
    std::mutex mutex;
    std::condition_variable cv;
    int arrived = 0;
    bool go = false;
    bool cancel = false;

    bool arrive_and_wait() {
        std::unique_lock<std::mutex> lock(mutex);
        if (++arrived >= 2) {
            go = true;
            cv.notify_all();
        } else {
            cv.wait(lock, [&] { return go || cancel; });
        }
        return !cancel;
    }

    void abort() {
        std::lock_guard<std::mutex> lock(mutex);
        cancel = true;
        cv.notify_all();
    }
};

std::mutex g_log_mutex;

void log_line(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    std::cout << line << '\n';
}

void log_err(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    std::cerr << line << '\n';
}

const char* motion_name(MotionKind motion) {
    switch (motion) {
        case MotionKind::Hold: return "hold";
        case MotionKind::MoveJ: return "movej";
        case MotionKind::MoveL: return "movel";
    }
    return "?";
}

bool parse_motion(const std::string& text, MotionKind& motion) {
    if (text == "hold") {
        motion = MotionKind::Hold;
        return true;
    }
    if (text == "movej") {
        motion = MotionKind::MoveJ;
        return true;
    }
    if (text == "movel") {
        motion = MotionKind::MoveL;
        return true;
    }
    return false;
}

bool parse_floats(const std::string& text, std::vector<float>& out) {
    out.clear();
    std::stringstream stream(text);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (item.empty()) return false;
        char* end = nullptr;
        const float value = std::strtof(item.c_str(), &end);
        if (end == item.c_str() || *end != '\0') return false;
        out.push_back(value);
    }
    return !out.empty();
}

bool assign_n(const std::vector<float>& values, float* dest, size_t count,
              const std::string& name) {
    if (values.size() != count) {
        log_err(name + " needs " + std::to_string(count) + " comma-separated numbers");
        return false;
    }
    for (size_t i = 0; i < count; ++i) dest[i] = values[i];
    return true;
}

std::string format_floats(const float* values, size_t count, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    for (size_t i = 0; i < count; ++i) {
        if (i) oss << ", ";
        oss << values[i];
    }
    return oss.str();
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
        for (float& component : q) component /= n;
    }
    return q;
}

/** 与 test_movel.cpp 相同：位置加增量，姿态右乘工具系 XYZ 欧拉增量。 */
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

std::string pose_text(const std::vector<float>& pose) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "xyz=(" << pose[0] << ", " << pose[1] << ", " << pose[2] << ") quat=("
        << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6] << ")";
    return oss.str();
}

bool enable_arm(ControlApi& api, const ArmJob& job) {
    const RetCode arm = api.set_group(job.group, true);
    if (arm != RetCode::SUCCESS) {
        log_err("[" + job.label + "] set_group(" + group_name(job.group) +
                ", true) -> " + ret_code_name(arm));
        return false;
    }
    (void)api.set_group(Group::HEAD, false);
    return true;
}

bool run_move_joint(ControlApi& api, const ArmJob& job, uint16_t speed_ratio) {
    std::vector<float> target;
    target.reserve(7);
    for (float degree : job.joints_deg) target.push_back(degree * kDegToRad);
    log_line("[" + job.label + "] move_joint target (deg): " +
             format_floats(job.joints_deg.data(), 7, 3) +
             " speed_ratio=" + std::to_string(speed_ratio));
    const RetCode code = api.move_joint(job.group, target, false, 0, 0, speed_ratio);
    if (code != RetCode::SUCCESS) {
        log_err("[" + job.label + "] move_joint -> " + ret_code_name(code));
        return false;
    }
    return true;
}

bool run_move_line(ControlApi& api, const ArmJob& job, const MotionLimits& limits) {
    const auto [code, pose] = api.get_tool_pose(job.group);
    if (code != RetCode::SUCCESS || pose.size() < 7) {
        log_err("[" + job.label + "] get_tool_pose -> " + ret_code_name(code));
        return false;
    }
    const std::array<float, 3> euler_rad = {job.drpy_deg[0] * kDegToRad,
                                             job.drpy_deg[1] * kDegToRad,
                                             job.drpy_deg[2] * kDegToRad};
    const std::vector<float> target = offset_pose(pose, job.dpos_m, euler_rad);
    log_line("[" + job.label + "] current " + pose_text(pose));
    log_line("[" + job.label + "] move_line dpos(m)=" +
             format_floats(job.dpos_m.data(), 3, 3) + " drpy(deg)=" +
             format_floats(job.drpy_deg.data(), 3, 1) +
             " speed=" + std::to_string(limits.line_speed) +
             " acc=" + std::to_string(limits.line_acc_s));
    log_line("[" + job.label + "] target " + pose_text(target));
    const RetCode line =
        api.move_line(job.group, target, limits.line_speed, limits.line_acc_s);
    if (line != RetCode::SUCCESS) {
        log_err("[" + job.label + "] move_line -> " + ret_code_name(line));
        return false;
    }
    return true;
}

void run_arm(ArmJob job, const MotionLimits& limits, StartGate& gate,
             std::atomic<bool>& ok) {
    try {
        log_line("[" + job.label + "] " + motion_name(job.motion) + "  ControlApi(" +
                 job.config_prefix + ") " + group_name(job.group));
        ControlApi api(job.config_prefix);
        std::vector<float> current;
        if (!wait_for_joint_state(api, job.group, current, kStateTimeoutMs)) {
            log_err("[" + job.label + "] wait joint state timeout");
            ok.store(false);
            gate.abort();
            return;
        }
        if (!enable_arm(api, job)) {
            ok.store(false);
            gate.abort();
            return;
        }
        if (!gate.arrive_and_wait()) {
            log_err("[" + job.label + "] cancelled before motion");
            ok.store(false);
            return;
        }
        bool moved = true;
        if (job.motion == MotionKind::Hold) {
            log_line("[" + job.label + "] hold");
        } else if (job.motion == MotionKind::MoveJ) {
            moved = run_move_joint(api, job, limits.joint_speed);
        } else {
            moved = run_move_line(api, job, limits);
        }
        if (!moved) {
            ok.store(false);
            return;
        }
        log_line("[" + job.label + "] done");
    } catch (const std::exception& ex) {
        log_err("[" + job.label + "] exception: " + ex.what());
        ok.store(false);
        gate.abort();
    }
}

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " <config_left> <config_right> [options]\n"
        << "  --left  movej|movel|hold     default movej, commands LEFT_ARM\n"
        << "  --right movej|movel|hold     default movej, commands RIGHT_ARM\n"
        << "  --joints-left/--joints-right j1..j7 degrees\n"
        << "      default " << format_floats(kDefaultJointsDeg, 7, 3) << "\n"
        << "  --dpos-left/--dpos-right     dx,dy,dz meters in the tool frame\n"
        << "      default " << format_floats(kDefaultDposM, 3, 2) << "\n"
        << "  --drpy-left/--drpy-right     rx,ry,rz degrees, tool-frame XYZ\n"
        << "      default " << format_floats(kDefaultDrpyDeg, 3, 1) << "\n"
        << "  --speed-joint 0..100         default " << kDefaultJointSpeed
        << "; 0 keeps the full joint velocity limit\n"
        << "  --speed-line  0..100         default " << kDefaultLineSpeed << "\n"
        << "  --acc-line seconds           default " << kDefaultLineAccS
        << "; 0 uses config.yaml acc_time\n";
}

bool take_arg(int argc, char* argv[], int& index, const char* name, std::string& value) {
    if (index + 1 >= argc) {
        log_err(std::string(name) + " needs a value");
        return false;
    }
    value = argv[++index];
    return true;
}

bool parse_args(int argc, char* argv[], ArmJob& left, ArmJob& right, MotionLimits& limits,
                bool& help_only) {
    help_only = false;
    std::vector<std::string> positionals;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        std::string value;
        if (arg == "--help" || arg == "-h") {
            help_only = true;
            continue;
        }
        if (arg == "--left" || arg == "--right") {
            if (!take_arg(argc, argv, i, arg.c_str(), value)) return false;
            MotionKind motion = MotionKind::Hold;
            if (!parse_motion(value, motion)) {
                log_err(arg + " must be movej, movel, or hold");
                return false;
            }
            (arg == "--left" ? left : right).motion = motion;
            continue;
        }
        auto parse_arm_floats = [&](float* dest, size_t count) {
            if (!take_arg(argc, argv, i, arg.c_str(), value)) return false;
            std::vector<float> numbers;
            if (!parse_floats(value, numbers)) {
                log_err(arg + " is not a comma-separated number list");
                return false;
            }
            if (!assign_n(numbers, dest, count, arg)) return false;
            return true;
        };
        if (arg == "--joints-left") {
            if (!parse_arm_floats(left.joints_deg.data(), 7)) return false;
        } else if (arg == "--joints-right") {
            if (!parse_arm_floats(right.joints_deg.data(), 7)) return false;
        } else if (arg == "--dpos-left") {
            if (!parse_arm_floats(left.dpos_m.data(), 3)) return false;
        } else if (arg == "--dpos-right") {
            if (!parse_arm_floats(right.dpos_m.data(), 3)) return false;
        } else if (arg == "--drpy-left") {
            if (!parse_arm_floats(left.drpy_deg.data(), 3)) return false;
        } else if (arg == "--drpy-right") {
            if (!parse_arm_floats(right.drpy_deg.data(), 3)) return false;
        } else if (arg == "--speed-joint" || arg == "--speed-line" || arg == "--acc-line") {
            if (!take_arg(argc, argv, i, arg.c_str(), value)) return false;
            char* end = nullptr;
            const float number = std::strtof(value.c_str(), &end);
            if (end == value.c_str() || *end != '\0') {
                log_err(arg + " is not a number");
                return false;
            }
            if (arg == "--acc-line") {
                if (number < 0.0F) {
                    log_err("--acc-line must be >= 0");
                    return false;
                }
                limits.line_acc_s = number;
            } else if (number < 0.0F || number > 100.0F) {
                log_err(arg + " must be 0..100");
                return false;
            } else if (arg == "--speed-joint") {
                limits.joint_speed = static_cast<uint16_t>(number);
            } else {
                limits.line_speed = static_cast<uint16_t>(number);
            }
        } else if (arg.rfind("--", 0) == 0) {
            log_err("unknown option " + arg);
            return false;
        } else {
            positionals.push_back(arg);
        }
    }
    if (help_only || positionals.size() != 2) {
        print_usage(argv[0]);
        return help_only && positionals.empty();
    }
    left.config_prefix = positionals[0];
    right.config_prefix = positionals[1];
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    ArmJob left{"left_rt", "", Group::LEFT_ARM};
    ArmJob right{"right_rt", "", Group::RIGHT_ARM};
    MotionLimits limits;
    bool help_only = false;
    if (!parse_args(argc, argv, left, right, limits, help_only)) return 2;
    if (help_only) return 0;

    log_line(std::string("left ") + motion_name(left.motion) + ", right " +
             motion_name(right.motion));

    StartGate gate;
    std::atomic<bool> ok{true};
    std::thread t_left(run_arm, left, std::cref(limits), std::ref(gate), std::ref(ok));
    std::thread t_right(run_arm, right, std::cref(limits), std::ref(gate), std::ref(ok));
    t_left.join();
    t_right.join();

    if (!ok.load()) {
        log_err("One or both arms failed.");
        return 1;
    }
    log_line("Both arms finished.");
    return 0;
}
