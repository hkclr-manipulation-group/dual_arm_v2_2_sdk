/**
 * @file dual_arm_independent_move_line.cpp
 * @brief 低跟随双臂独立 MoveL：set_left/right_tool_line_target + auto flush。
 *
 * 不用 move_line(ALL, 14)。SDK 后台线程将左右 Tool 目标合并在**同一 UDP 包**
 * 里发 kTaskLine，并阻塞等待 send_task_with_ack（低跟随 quintic MoveL）。
 *
 * 流程：使能 L/R、关头 → start_dual_arm_tool_line_auto_flush →
 *       连续 set_left / set_right（不同目标）→ 等待到位 → 再 set 回起始。
 *
 * 目标 3D ≤1 mm：轮询 ToolPose + 合包 MoveL 未达标时降速重发（最多 3 次）。
 *
 * 用法：./dual_arm_independent_move_line [config_prefix_path]
 *
 * 每次退出（含失败）打印 Run error summary，并追加 CSV（默认可执行文件同目录；
 * 可用环境变量 DUAL_ARM_MOVEL_ERROR_CSV 指定路径）。
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

constexpr float kLeftDeltaXM = 0.30F;
constexpr float kLeftDeltaYM = 0.07F;
constexpr float kLeftDeltaZM = -0.08F;
constexpr float kRightDeltaZM = -0.06F;
constexpr uint16_t kSpeedRatio = 15;
constexpr float kAccTimeS = 4.0F;
constexpr uint16_t kRetrySpeedRatio = 10;
constexpr float kRetryAccTimeS = 5.0F;
constexpr int kCoalesceMs = 15;
constexpr float kPosToleranceM = 0.001F;
constexpr float kMaxSpeedDegS = 12.0F;
constexpr int kMoveTimeoutMs = 40000;
constexpr int kPollMs = 50;
constexpr int kPoseStableCycles = 5;
constexpr int kMaxMoveLineAttempts = 3;
constexpr const char* kErrorCsvName =
    "dual_arm_independent_move_line_3d_errors_mm.csv";

struct PoseError3d {
    float err_m = 0.0F;
    float dx_m = 0.0F;
    float dy_m = 0.0F;
    float dz_m = 0.0F;
    bool reached = false;
};

struct PhaseReach {
    PoseError3d left{};
    PoseError3d right{};
};

struct RunErrorLog {
    PhaseReach forward{};
    PhaseReach return_home{};
    PoseError3d final_left_vs_start{};
    PoseError3d final_right_vs_start{};
    bool measured_forward = false;
    bool measured_return = false;
    bool measured_final = false;
    bool run_ok = false;
    /** 非空表示未跑完主流程（如未连上 rt_control）。 */
    std::string abort_reason;
};

std::vector<float> offset_xyz(const std::vector<float>& pose, float dx, float dy,
                              float dz) {
    std::vector<float> out = pose;
    if (out.size() >= 3) {
        out[0] += dx;
        out[1] += dy;
        out[2] += dz;
    }
    return out;
}

std::vector<float> offset_z(const std::vector<float>& pose, float dz) {
    return offset_xyz(pose, 0.0F, 0.0F, dz);
}

float position_error_m(const std::vector<float>& a, const std::vector<float>& b) {
    if (a.size() < 3 || b.size() < 3) return 1e9F;
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

PoseError3d pose_error_3d(const std::vector<float>& current,
                          const std::vector<float>& target,
                          bool reached) {
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
    // 默认可执行文件同目录，避免从 build/bin/cpp 运行时写到不易发现的相对路径。
    const fs::path beside_exe = fs::weakly_canonical(exe_dir, ec) / kErrorCsvName;
    if (!ec) {
        return beside_exe;
    }
    return exe_dir / kErrorCsvName;
}

float err_mm_value(const PoseError3d& e) { return e.err_m * 1000.0F; }

void csv_append_err_mm(std::ostream& out, bool measured, const PoseError3d& e) {
    out << ',';
    if (!measured) return;
    out << std::fixed << std::setprecision(3) << err_mm_value(e);
}

bool append_run_errors_csv(const std::filesystem::path& path,
                           const RunErrorLog& log) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path parent = path.parent_path();
    if (!parent.empty()) {
        fs::create_directories(parent, ec);
    }

    const bool write_header = !fs::exists(path, ec) || fs::file_size(path, ec) == 0;
    std::ofstream out(path, std::ios::app);
    if (!out) {
        std::cerr << "Failed to open CSV for append: " << path << '\n';
        return false;
    }

    if (write_header) {
        out << "time,fwd_L_mm,fwd_R_mm,ret_L_mm,ret_R_mm,final_L_mm,final_R_mm\n";
    }

    out << local_timestamp_iso8601();
    csv_append_err_mm(out, log.measured_forward, log.forward.left);
    csv_append_err_mm(out, log.measured_forward, log.forward.right);
    csv_append_err_mm(out, log.measured_return, log.return_home.left);
    csv_append_err_mm(out, log.measured_return, log.return_home.right);
    csv_append_err_mm(out, log.measured_final, log.final_left_vs_start);
    csv_append_err_mm(out, log.measured_final, log.final_right_vs_start);
    out << '\n';
    out.flush();
    if (!out.good()) {
        std::cerr << "CSV write failed: " << path << '\n';
        return false;
    }
    return true;
}

void print_run_error_summary(const RunErrorLog& log) {
    const float tol_mm = kPosToleranceM * 1000.0F;
    std::cout << "\n======== Tool 3D 位置误差 (mm)，容差 " << std::fixed
              << std::setprecision(1) << tol_mm << " ========\n";
    if (!log.abort_reason.empty()) {
        std::cout << "  说明: " << log.abort_reason << "（下列未测项无数据）\n";
    }
    std::cout << "  去程 左臂: ";
    if (log.measured_forward) {
        std::cout << std::setprecision(3) << err_mm_value(log.forward.left)
                  << " mm    右臂: " << err_mm_value(log.forward.right)
                  << " mm\n";
    } else {
        std::cout << "(未测)\n";
    }
    std::cout << "  回程 左臂: ";
    if (log.measured_return) {
        std::cout << std::setprecision(3) << err_mm_value(log.return_home.left)
                  << " mm    右臂: " << err_mm_value(log.return_home.right)
                  << " mm\n";
    } else {
        std::cout << "(未测)\n";
    }
    std::cout << "  终点 左臂: ";
    if (log.measured_final) {
        std::cout << std::setprecision(3)
                  << err_mm_value(log.final_left_vs_start)
                  << " mm    右臂: "
                  << err_mm_value(log.final_right_vs_start) << " mm  (相对起点)\n";
    } else {
        std::cout << "(未测)\n";
    }
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

/** 进程退出前必定写汇总 + CSV（含提前失败）。 */
class RunLogFinalizer {
public:
    std::string config_prefix;
    std::filesystem::path csv_path;
    RunErrorLog log;

    explicit RunLogFinalizer(std::string config)
        : config_prefix(std::move(config)), csv_path(error_csv_path()) {
        std::cout << "dual_arm_independent_move_line: 3D误差 CSV 累积 -> "
                  << csv_path << '\n';
        std::cout.flush();
    }

    ~RunLogFinalizer() {
        if (!emitted_) {
            if (log.abort_reason.empty()) {
                log.abort_reason = "exit_before_finalize";
            }
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
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "  position (m): " << pose[0] << ", " << pose[1] << ", "
              << pose[2] << '\n';
    std::cout << "  quaternion:   " << pose[3] << ", " << pose[4] << ", "
              << pose[5] << ", " << pose[6] << '\n';
}

void print_pose_3d_error_mm(const char* arm_label,
                            const std::vector<float>& current,
                            const std::vector<float>& target) {
    const float err_m = position_error_m(current, target);
    const float err_mm = err_m * 1000.0F;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  " << arm_label << " 3D error vs target: " << err_mm
              << " mm (tolerance " << kPosToleranceM * 1000.0F << " mm)";
    if (err_m > kPosToleranceM) {
        std::cout << "  WARNING: over tolerance";
    }
    std::cout << '\n';
    if (current.size() >= 3 && target.size() >= 3) {
        std::cout << std::setprecision(4);
        std::cout << "    dX=" << (current[0] - target[0]) * 1000.0F << " mm, dY="
                  << (current[1] - target[1]) * 1000.0F
                  << " mm, dZ=" << (current[2] - target[2]) * 1000.0F << " mm\n";
    }
}

bool setup_both_arm_speed(ControlApi& api) {
    const std::vector<float> speed(7, kMaxSpeedDegS * kDegToRad);
    return api.set_joint_max_speed(Group::LEFT_ARM, speed).size() == 7 &&
           api.set_joint_max_speed(Group::RIGHT_ARM, speed).size() == 7;
}

/** 与 test_move_line 相同思路：3D 位置误差 + 连续稳定周期。 */
bool wait_both_tool_targets(ControlApi& api,
                            const std::vector<float>& left_target,
                            const std::vector<float>& right_target,
                            int timeout_ms,
                            float* last_left_err_m,
                            float* last_right_err_m) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    int stable = 0;
    float left_err = 1e9F;
    float right_err = 1e9F;

    while (std::chrono::steady_clock::now() < deadline) {
        auto [l_code, left_now] = api.get_tool_pose(Group::LEFT_ARM);
        auto [r_code, right_now] = api.get_tool_pose(Group::RIGHT_ARM);
        if (l_code != RetCode::SUCCESS || r_code != RetCode::SUCCESS ||
            left_now.size() < 7 || right_now.size() < 7) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
            continue;
        }
        left_err = position_error_m(left_now, left_target);
        right_err = position_error_m(right_now, right_target);
        if (left_err <= kPosToleranceM && right_err <= kPosToleranceM) {
            if (++stable >= kPoseStableCycles) {
                if (last_left_err_m) *last_left_err_m = left_err;
                if (last_right_err_m) *last_right_err_m = right_err;
                return true;
            }
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
    }

    if (last_left_err_m) *last_left_err_m = left_err;
    if (last_right_err_m) *last_right_err_m = right_err;
    return false;
}

PhaseReach report_dual_reach(ControlApi& api, const char* phase,
                             const std::vector<float>& left_target,
                             const std::vector<float>& right_target,
                             bool reached) {
    PhaseReach report;
    report.left.reached = reached;
    report.right.reached = reached;

    auto [l_code, left_now] = api.get_tool_pose(Group::LEFT_ARM);
    auto [r_code, right_now] = api.get_tool_pose(Group::RIGHT_ARM);
    if (l_code != RetCode::SUCCESS || r_code != RetCode::SUCCESS ||
        left_now.size() < 7 || right_now.size() < 7) {
        std::cerr << phase << ": get_tool_pose failed after wait\n";
        report.left.reached = false;
        report.right.reached = false;
        return report;
    }

    report.left = pose_error_3d(left_now, left_target, reached);
    report.right = pose_error_3d(right_now, right_target, reached);

    std::cout << "\n--- " << phase << " ---\n";
    print_tool_pose("  Left actual:", left_now);
    print_pose_3d_error_mm("Left", left_now, left_target);
    print_tool_pose("  Right actual:", right_now);
    print_pose_3d_error_mm("Right", right_now, right_target);

    if (!reached) {
        std::cout << "Warning: auto-flush ACK ok but ToolPose did not reach "
                     "target within "
                  << kMoveTimeoutMs << " ms ("
                  << group_name(Group::LEFT_ARM) << " + "
                  << group_name(Group::RIGHT_ARM) << ")\n";
        return report;
    }
    std::cout << phase << ": reached within tolerance\n";
    return report;
}

/** 连续 set 左右目标，让 coalesce 合并成一次低跟随 MoveL。 */
bool submit_both_targets(ControlApi& api, const std::vector<float>& left,
                         const std::vector<float>& right) {
    if (api.set_left_tool_line_target(left) != RetCode::SUCCESS) return false;
    if (api.set_right_tool_line_target(right) != RetCode::SUCCESS) return false;
    return true;
}

bool restart_auto_flush(ControlApi& api, uint16_t speed_ratio, float acc_time_s) {
    api.stop_dual_arm_tool_line_auto_flush();
    return api.start_dual_arm_tool_line_auto_flush(speed_ratio, acc_time_s,
                                                   kCoalesceMs) == RetCode::SUCCESS;
}

PhaseReach wait_after_submit(ControlApi& api, const char* phase,
                             const std::vector<float>& left_target,
                             const std::vector<float>& right_target) {
    std::cout << phase << ": polling ToolPose (timeout " << kMoveTimeoutMs
              << " ms)...\n"
              << std::flush;
    float left_err = 0.0F;
    float right_err = 0.0F;
    const bool reached = wait_both_tool_targets(
        api, left_target, right_target, kMoveTimeoutMs, &left_err, &right_err);
    if (!reached) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  last poll 3D err: L " << left_err * 1000.0F << " mm, R "
                  << right_err * 1000.0F << " mm\n";
    }
    return report_dual_reach(api, phase, left_target, right_target, reached);
}

bool phase_fully_reached(const PhaseReach& phase) {
    return phase.left.reached && phase.right.reached &&
           phase.left.err_m <= kPosToleranceM &&
           phase.right.err_m <= kPosToleranceM;
}

/** set L/R + 等待双臂 1 mm；未达标则 stop/start 更慢 profile 并重发，最多 3 次。 */
bool submit_phase_until_reach(ControlApi& api, const char* phase,
                              const std::vector<float>& left_target,
                              const std::vector<float>& right_target,
                              PhaseReach& out) {
    PhaseReach last;
    for (int attempt = 1; attempt <= kMaxMoveLineAttempts; ++attempt) {
        if (attempt > 1) {
            std::cout << phase << ": 未进 1 mm 容差，重发合包 MoveL " << attempt
                      << '/' << kMaxMoveLineAttempts << " (更慢 profile)\n";
        }
        const uint16_t ratio =
            (attempt == 1) ? kSpeedRatio : kRetrySpeedRatio;
        const float acc = (attempt == 1) ? kAccTimeS : kRetryAccTimeS;
        if (!restart_auto_flush(api, ratio, acc)) {
            std::cerr << phase << ": start_dual_arm_tool_line_auto_flush failed\n";
            return false;
        }
        if (!submit_both_targets(api, left_target, right_target)) {
            std::cerr << phase << ": set_*_tool_line_target failed\n";
            return false;
        }
        last = wait_after_submit(api, phase, left_target, right_target);
        if (phase_fully_reached(last)) {
            out = last;
            return true;
        }
    }
    out = last;
    std::cout << "ERROR: " << phase << " 双臂 3D 仍 > "
              << (kPosToleranceM * 1000.0F) << " mm after "
              << kMaxMoveLineAttempts << " attempts\n";
    return false;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string config_prefix = config_path_from_args(argc, argv);
    RunLogFinalizer run_finalizer(config_prefix);
    RunErrorLog& run_log = run_finalizer.log;

    try {
        ControlApi api(config_prefix);

        std::vector<float> probe;
        if (!wait_for_joint_state(api, Group::LEFT_ARM, probe)) {
            std::cerr << "No rt_control state within 3 s.\n";
            run_log.abort_reason = "no_rt_control_state";
            run_finalizer.finalize();
            return 1;
        }

        if (api.set_group(Group::LEFT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::RIGHT_ARM, true) != RetCode::SUCCESS ||
            api.set_group(Group::HEAD, false) != RetCode::SUCCESS) {
            std::cerr << "Failed to enable L/R (head disabled for MoveL).\n";
            run_log.abort_reason = "enable_group_failed";
            run_finalizer.finalize();
            return 1;
        }
        if (!setup_both_arm_speed(api)) {
            std::cerr << "set_joint_max_speed failed.\n";
            run_log.abort_reason = "set_joint_max_speed_failed";
            run_finalizer.finalize();
            return 1;
        }

        auto [l0_code, left_start] = api.get_tool_pose(Group::LEFT_ARM);
        auto [r0_code, right_start] = api.get_tool_pose(Group::RIGHT_ARM);
        if (l0_code != RetCode::SUCCESS || r0_code != RetCode::SUCCESS ||
            left_start.size() != 7 || right_start.size() != 7) {
            std::cerr << "get_tool_pose failed.\n";
            run_log.abort_reason = "get_tool_pose_failed";
            run_finalizer.finalize();
            return 1;
        }

        std::cout << "Low-follow dual MoveL: set_left/right + auto flush\n"
                  << "  1 mm 容差, coalesce_ms=" << kCoalesceMs
                  << "  speed_ratio=" << kSpeedRatio
                  << "  acc_time=" << kAccTimeS << " s"
                  << " (retry ratio=" << kRetrySpeedRatio << " acc="
                  << kRetryAccTimeS << " s, max " << kMaxMoveLineAttempts
                  << "x)\n"
                  << "  move_timeout=" << kMoveTimeoutMs << " ms\n"
                  << "  LEFT dX/dY/dZ=" << kLeftDeltaXM << '/' << kLeftDeltaYM
                  << '/' << kLeftDeltaZM << " m, RIGHT dZ=" << kRightDeltaZM
                  << " m\n";

        print_tool_pose("Start LEFT ToolPose:", left_start);
        print_tool_pose("Start RIGHT ToolPose:", right_start);

        const std::vector<float> left_fwd =
            offset_xyz(left_start, kLeftDeltaXM, kLeftDeltaYM, kLeftDeltaZM);
        const std::vector<float> right_fwd =
            offset_z(right_start, kRightDeltaZM);

        std::cout << "\n==> Forward: set_left then set_right (one coalesced MoveL)\n";
        print_tool_pose("  Target LEFT:", left_fwd);
        print_tool_pose("  Target RIGHT:", right_fwd);
        bool ok = submit_phase_until_reach(
            api, "Forward reach", left_fwd, right_fwd, run_log.forward);
        run_log.measured_forward = true;
        if (!ok) {
            api.stop_dual_arm_tool_line_auto_flush();
            run_log.abort_reason = "forward_not_within_1mm";
            run_log.run_ok = false;
            run_finalizer.finalize();
            return 1;
        }

        std::cout << "\n==> Return: set_left/right back to start ToolPose\n";
        print_tool_pose("  Target LEFT (start):", left_start);
        print_tool_pose("  Target RIGHT (start):", right_start);
        ok = submit_phase_until_reach(api, "Return reach", left_start,
                                      right_start, run_log.return_home);
        run_log.measured_return = true;
        if (!ok) {
            api.stop_dual_arm_tool_line_auto_flush();
            run_log.abort_reason = "return_not_within_1mm";
            run_log.run_ok = false;
            run_finalizer.finalize();
            return 1;
        }

        api.stop_dual_arm_tool_line_auto_flush();

        auto [lf, left_final] = api.get_tool_pose(Group::LEFT_ARM);
        auto [rf, right_final] = api.get_tool_pose(Group::RIGHT_ARM);
        if (lf == RetCode::SUCCESS && rf == RetCode::SUCCESS) {
            std::cout << "\n--- Final vs start ---\n";
            print_pose_3d_error_mm("Left", left_final, left_start);
            print_pose_3d_error_mm("Right", right_final, right_start);
            run_log.final_left_vs_start = pose_error_3d(left_final, left_start,
                position_error_m(left_final, left_start) <= kPosToleranceM);
            run_log.final_right_vs_start = pose_error_3d(
                right_final, right_start,
                position_error_m(right_final, right_start) <= kPosToleranceM);
            run_log.measured_final = true;
        }

        run_log.run_ok = ok;
        run_finalizer.finalize();

        if (!ok) {
            std::cerr << "\nDone with reach warnings (see above).\n";
            return 1;
        }
        std::cout << "\nAll done.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fatal: " << error.what() << '\n';
        run_log.abort_reason = std::string("exception:") + error.what();
        run_finalizer.finalize();
        return 1;
    }
}
