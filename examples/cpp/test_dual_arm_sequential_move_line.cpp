/**
 * @file test_dual_arm_sequential_move_line.cpp
 * @brief 顺序双臂 MoveL
 *
 * 左臂 Z 去程 + 回到起点 ToolPose → 右臂 Z 去程 + 回到起点（另一臂由控制器保持）。
 * 目标 3D 误差 ≤1 mm：降速 MoveL + 轮询到位，未达标最多 3 次重发；CSV 累积。
 *
 * 用法：./test_dual_arm_sequential_move_line [config_prefix_path]
 */

#include "control_api.h"
#include "example_common.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using dual_arm_v2_2_sdk::ControlApi;
using dual_arm_v2_2_sdk::Group;
using dual_arm_v2_2_sdk::RetCode;
using example::config_path_from_args;
using example::group_name;
using example::kDegToRad;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr float kLeftDeltaZM = 0.08F;
constexpr float kRightDeltaZM = -0.06F;
constexpr uint16_t kSpeedRatio = 15;
constexpr float kAccTimeS = 4.0F;
constexpr uint16_t kRetrySpeedRatio = 10;
constexpr float kRetryAccTimeS = 5.0F;
constexpr float kPosToleranceM = 0.001F;
constexpr float kMaxSpeedDegS = 12.0F;
constexpr int kMoveTimeoutMs = 40000;
constexpr int kPollMs = 50;
constexpr int kPoseStableCycles = 5;
constexpr int kMaxMoveLineAttempts = 3;
constexpr const char* kErrorCsvName =
    "test_dual_arm_sequential_move_line_3d_errors_mm.csv";

struct PoseError3d {
    float err_m = 0.0F;
    float dx_m = 0.0F;
    float dy_m = 0.0F;
    float dz_m = 0.0F;
    bool reached = false;
};

struct RunErrorLog {
    PoseError3d fwd_L{};
    PoseError3d fwd_R{};
    PoseError3d ret_L{};
    PoseError3d ret_R{};
    PoseError3d final_L{};
    PoseError3d final_R{};
    bool has_fwd_L = false;
    bool has_fwd_R = false;
    bool has_ret_L = false;
    bool has_ret_R = false;
    bool has_final = false;
    bool run_ok = false;
    std::string abort_reason;
};

std::vector<float> offset_z(const std::vector<float>& pose, float dz) {
    std::vector<float> out = pose;
    if (out.size() >= 3) out[2] += dz;
    return out;
}

float position_error_m(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() < 3 || b.size() < 3) return 1e9F;
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

PoseError3d pose_error_3d(const std::vector<float>& current,
                          const std::vector<float>& target, bool reached) {
    PoseError3d out;
    out.reached = reached;
    if (current.size() < 3 || target.size() < 3) {
        out.err_m = 1e9F;
        return out;
    }
    out.dx_m = current[0] - target[0];
    out.dy_m = current[1] - target[1];
    out.dz_m = current[2] - target[2];
    out.err_m = std::sqrt(out.dx_m * out.dx_m + out.dy_m * out.dy_m +
                          out.dz_m * out.dz_m);
    return out;
}

std::string local_timestamp_iso8601() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_local{};
#if defined(_WIN32)
    localtime_s(&tm_local, &t);
#else
    localtime_r(&t, &tm_local);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_local, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

std::filesystem::path error_csv_path() {
    namespace fs = std::filesystem;
    if (const char* env = std::getenv("DUAL_ARM_MOVEL_ERROR_CSV")) {
        return fs::path(env);
    }
    const fs::path exe_dir = example::executable_directory();
    std::error_code ec;
    const fs::path beside_exe = fs::weakly_canonical(exe_dir, ec) / kErrorCsvName;
    if (!ec) return beside_exe;
    return exe_dir / kErrorCsvName;
}

float err_mm(const PoseError3d& e) { return e.err_m * 1000.0F; }

void csv_append_err_mm(std::ostream& out, bool measured, const PoseError3d& e) {
    out << ',';
    if (!measured) return;
    out << std::fixed << std::setprecision(3) << err_mm(e);
}

bool append_run_errors_csv(const std::filesystem::path& path,
                           const RunErrorLog& log) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!path.parent_path().empty()) {
        fs::create_directories(path.parent_path(), ec);
    }
    const bool write_header =
        !fs::exists(path, ec) || fs::file_size(path, ec) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out) {
        std::cerr << "Failed to open CSV: " << path << '\n';
        return false;
    }
    if (write_header) {
        out << "time,fwd_L_mm,fwd_R_mm,ret_L_mm,ret_R_mm,final_L_mm,final_R_mm\n";
    }
    out << local_timestamp_iso8601();
    csv_append_err_mm(out, log.has_fwd_L, log.fwd_L);
    csv_append_err_mm(out, log.has_fwd_R, log.fwd_R);
    csv_append_err_mm(out, log.has_ret_L, log.ret_L);
    csv_append_err_mm(out, log.has_ret_R, log.ret_R);
    csv_append_err_mm(out, log.has_final, log.final_L);
    csv_append_err_mm(out, log.has_final, log.final_R);
    out << '\n';
    out.flush();
    return static_cast<bool>(out);
}

void print_run_error_summary(const RunErrorLog& log) {
    const float tol_mm = kPosToleranceM * 1000.0F;
    std::cout << "\n======== Tool 3D 位置误差 (mm)，容差 " << std::fixed
              << std::setprecision(1) << tol_mm << " ========\n";
    if (!log.abort_reason.empty()) {
        std::cout << "  说明: " << log.abort_reason << '\n';
    }
    auto line = [tol_mm](const char* phase, const char* arm, bool has,
                         const PoseError3d& e) {
        std::cout << "  " << phase << " " << arm << ": ";
        if (!has) {
            std::cout << "(未测)\n";
            return;
        }
        std::cout << std::setprecision(3) << err_mm(e) << " mm";
        if (err_mm(e) > tol_mm) std::cout << "  WARNING";
        std::cout << '\n';
    };
    line("去程", "左臂", log.has_fwd_L, log.fwd_L);
    line("去程", "右臂", log.has_fwd_R, log.fwd_R);
    line("回程", "左臂", log.has_ret_L, log.ret_L);
    line("回程", "右臂", log.has_ret_R, log.ret_R);
    line("终点", "左臂", log.has_final, log.final_L);
    line("终点", "右臂", log.has_final, log.final_R);
    std::cout << "  CSV 列: time,fwd_L_mm,fwd_R_mm,ret_L_mm,ret_R_mm,final_L_mm,"
                 "final_R_mm\n";
    std::cout.flush();
}

void emit_run_error_log(const std::filesystem::path& csv_path, RunErrorLog& log) {
    print_run_error_summary(log);
    if (append_run_errors_csv(csv_path, log)) {
        std::cout << "  已追加到 CSV: " << csv_path << '\n';
    } else {
        std::cerr << "  CSV 写入失败: " << csv_path << '\n';
    }
    std::cout.flush();
    std::cerr.flush();
}

class RunLogFinalizer {
public:
    std::filesystem::path csv_path;
    RunErrorLog log;

    RunLogFinalizer() : csv_path(error_csv_path()) {
        std::cout << "test_dual_arm_sequential_move_line: 3D误差 CSV -> "
                  << csv_path << '\n';
        std::cout.flush();
    }

    ~RunLogFinalizer() {
        if (!emitted_) {
            if (log.abort_reason.empty()) log.abort_reason = "exit_before_finalize";
            emit_run_error_log(csv_path, log);
        }
    }

    void finalize() {
        if (emitted_) return;
        emitted_ = true;
        emit_run_error_log(csv_path, log);
    }

private:
    bool emitted_ = false;
};

void print_tool_pose(const char* label, const std::vector<float>& pose) {
    if (pose.size() < 7) return;
    std::cout << label << '\n';
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  xyz (m): " << pose[0] << ", " << pose[1] << ", " << pose[2]
              << '\n';
    std::cout << "  quat:    " << pose[3] << ", " << pose[4] << ", " << pose[5]
              << ", " << pose[6] << '\n';
}

void print_pose_3d_error_mm(const char* label, const std::vector<float>& current,
                            const std::vector<float>& target) {
    const PoseError3d e =
        pose_error_3d(current, target, position_error_m(current, target) <= kPosToleranceM);
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  " << label << " 3D error: " << err_mm(e) << " mm";
    if (err_mm(e) > kPosToleranceM * 1000.0F) {
        std::cout << "  WARNING: over tolerance";
    }
    std::cout << "  (dX=" << e.dx_m * 1000.0F << " dY=" << e.dy_m * 1000.0F
              << " dZ=" << e.dz_m * 1000.0F << " mm)\n";
}

bool configure_arms(ControlApi& api) {
    if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
        api.set_group(Group::HEAD, false) != RetCode::SUCCESS) {
        std::cerr << "Failed to enable L/R (head disabled).\n";
        return false;
    }
    const std::vector<float> speed(7, kMaxSpeedDegS * kDegToRad);
    if (api.set_joint_max_speed(Group::LEFT_ARM, speed).size() != 7 ||
        api.set_joint_max_speed(Group::RIGHT_ARM, speed).size() != 7) {
        std::cerr << "set_joint_max_speed failed.\n";
        return false;
    }
    std::cout << "Arms: left+right enabled, head disabled; joint speed cap "
              << kMaxSpeedDegS << " deg/s\n";
    return true;
}

Group peer_arm_group(Group arm) {
    return arm == Group::LEFT_ARM ? Group::RIGHT_ARM : Group::LEFT_ARM;
}

bool hold_peer_arm_joints(ControlApi& api, Group moving_arm) {
    const Group peer = peer_arm_group(moving_arm);
    auto [code, joints] = api.get_joint(peer);
    if (code != RetCode::SUCCESS || joints.size() != 7) return false;
    return api.move_joint(peer, joints, false) == RetCode::SUCCESS;
}

bool wait_tool_pose_3d(ControlApi& api, Group group,
                       const std::vector<float>& target, int timeout_ms,
                       float* last_err_m) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    float err = 1e9F;
    while (std::chrono::steady_clock::now() < deadline) {
        auto [code, now] = api.get_tool_pose(group);
        if (code != RetCode::SUCCESS || now.size() < 7) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            continue;
        }
        err = position_error_m(now, target);
        if (err <= kPosToleranceM) {
            if (++stable >= kPoseStableCycles) {
                if (last_err_m) *last_err_m = err;
                return true;
            }
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }
    if (last_err_m) *last_err_m = err;
    return false;
}

/** move_line + 等待 1 mm 内；未达标则降速重发，最多 kMaxMoveLineAttempts 次。 */
bool run_move_line_until_reach(ControlApi& api, Group group,
                               const std::vector<float>& target,
                               const char* step_label) {
    std::cout << step_label << '\n';
    if (target.size() >= 7) print_tool_pose("Target:", target);

    float last_err = 1e9F;
    for (int attempt = 1; attempt <= kMaxMoveLineAttempts; ++attempt) {
        if (attempt > 1) {
            std::cout << "  未进 1 mm 容差，重发 move_line " << attempt << '/'
                      << kMaxMoveLineAttempts << " (更慢)\n";
        }
        if (!hold_peer_arm_joints(api, group)) {
            std::cerr << "  hold peer arm failed\n";
            return false;
        }

        const uint16_t ratio =
            (attempt == 1) ? kSpeedRatio : kRetrySpeedRatio;
        const float acc_time =
            (attempt == 1) ? kAccTimeS : kRetryAccTimeS;

        const RetCode code = api.move_line(group, target, ratio, acc_time);
        std::cout << "move_line(" << group_name(group) << ", ratio=" << ratio
                  << ", acc=" << acc_time << "s): " << ret_code_name(code)
                  << '\n';
        if (code != RetCode::SUCCESS) return false;

        const bool reached = wait_tool_pose_3d(
            api, group, target, kMoveTimeoutMs, &last_err);
        auto [p_code, actual] = api.get_tool_pose(group);
        if (p_code == RetCode::SUCCESS && actual.size() >= 7 &&
            target.size() >= 7) {
            print_pose_3d_error_mm("After wait", actual, target);
        }
        if (reached) return true;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "ERROR: 3D 误差仍 > " << (kPosToleranceM * 1000.0F)
              << " mm after " << kMaxMoveLineAttempts
              << " attempts (last " << last_err * 1000.0F << " mm)\n";
    return false;
}

/** 单臂：去程 MoveL → 回到起点 ToolPose；写入 run_log 对应误差槽。 */
bool movel_out_and_back(ControlApi& api, Group group, float dz_m,
                        const char* section_title, RunErrorLog& run_log,
                        bool is_left) {
    std::cout << "\n======== " << section_title << " (dz=" << std::showpos
              << dz_m << " m) ========\n"
              << std::noshowpos;

    auto [s_code, start] = api.get_tool_pose(group);
    if (s_code != RetCode::SUCCESS || start.size() != 7) {
        std::cerr << "get_tool_pose failed for " << group_name(group) << '\n';
        return false;
    }
    print_tool_pose("Start ToolPose:", start);

    const std::vector<float> forward = offset_z(start, dz_m);
    if (!run_move_line_until_reach(api, group, forward, "MoveL forward...")) {
        return false;
    }

    auto [af_code, after_fwd] = api.get_tool_pose(group);
    if (af_code != RetCode::SUCCESS || after_fwd.size() < 7) return false;
    const bool fwd_ok =
        position_error_m(after_fwd, forward) <= kPosToleranceM;
    const PoseError3d fwd_err = pose_error_3d(after_fwd, forward, fwd_ok);
    if (is_left) {
        run_log.fwd_L = fwd_err;
        run_log.has_fwd_L = true;
    } else {
        run_log.fwd_R = fwd_err;
        run_log.has_fwd_R = true;
    }

    if (!run_move_line_until_reach(api, group, start,
                                   "MoveL return to start...")) {
        return false;
    }

    auto [ar_code, after_ret] = api.get_tool_pose(group);
    if (ar_code != RetCode::SUCCESS || after_ret.size() < 7) return false;
    const bool ret_ok = position_error_m(after_ret, start) <= kPosToleranceM;
    const PoseError3d ret_err = pose_error_3d(after_ret, start, ret_ok);
    if (is_left) {
        run_log.ret_L = ret_err;
        run_log.has_ret_L = true;
    } else {
        run_log.ret_R = ret_err;
        run_log.has_ret_R = true;
    }

    print_tool_pose("Final ToolPose:", after_ret);
    print_pose_3d_error_mm("Final vs start", after_ret, start);
    return fwd_ok && ret_ok;
}

}  // namespace

int main(int argc, char* argv[]) {
    RunLogFinalizer run_finalizer;
    RunErrorLog& run_log = run_finalizer.log;

    try {
        ControlApi api(config_path_from_args(argc, argv));

        std::vector<float> probe;
        if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
            std::cerr << "No rt_control state within 3 s.\n";
            run_log.abort_reason = "no_rt_control_state";
            run_finalizer.finalize();
            return 1;
        }

        if (!configure_arms(api)) {
            run_log.abort_reason = "configure_arms_failed";
            run_finalizer.finalize();
            return 1;
        }

        std::cout << "\nSequential dual MoveL (move_line per arm)\n"
                  << "  1 mm 容差, speed_ratio=" << kSpeedRatio
                  << " acc_time=" << kAccTimeS << " s"
                  << " (retry ratio=" << kRetrySpeedRatio << " acc="
                  << kRetryAccTimeS << " s, max " << kMaxMoveLineAttempts
                  << "x)\n"
                  << "Order: LEFT out/back -> RIGHT out/back.\n";

        auto [l0, left_start] = api.get_tool_pose(Group::LEFT_ARM);
        auto [r0, right_start] = api.get_tool_pose(Group::RIGHT_ARM);
        if (l0 != RetCode::SUCCESS || r0 != RetCode::SUCCESS ||
            left_start.size() != 7 || right_start.size() != 7) {
            run_log.abort_reason = "get_tool_pose_failed";
            run_finalizer.finalize();
            return 1;
        }

        bool ok = movel_out_and_back(api, Group::LEFT_ARM, kLeftDeltaZM,
                                     "Left arm", run_log, true);
        ok &= movel_out_and_back(api, Group::RIGHT_ARM, kRightDeltaZM,
                                 "Right arm", run_log, false);

        auto [lf, left_final] = api.get_tool_pose(Group::LEFT_ARM);
        auto [rf, right_final] = api.get_tool_pose(Group::RIGHT_ARM);
        if (lf == RetCode::SUCCESS && rf == RetCode::SUCCESS) {
            run_log.final_L = pose_error_3d(
                left_final, left_start,
                position_error_m(left_final, left_start) <= kPosToleranceM);
            run_log.final_R = pose_error_3d(
                right_final, right_start,
                position_error_m(right_final, right_start) <= kPosToleranceM);
            run_log.has_final = true;
        }

        run_log.run_ok = ok;
        run_finalizer.finalize();

        if (!ok) {
            std::cerr << "\nDone with reach warnings.\n";
            return 1;
        }
        std::cout << "\nAll done.\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Fatal: " << ex.what() << '\n';
        run_log.abort_reason = std::string("exception:") + ex.what();
        run_finalizer.finalize();
        return 1;
    }
}
