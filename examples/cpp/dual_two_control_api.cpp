/**
 * @file dual_two_control_api.cpp
 * @brief 两个 rt_control / 两套 port：各用一个 ControlApi 实例。
 *
 * 用法：
 *   ./dual_two_control_api <config_left> <config_right>
 *
 * 左 config → Group::LEFT_ARM (arm index 0)；右 config → Group::RIGHT_ARM (index 2)。
 */

#include "control_api.h"
#include "example_common.h"

#include <atomic>
#include <chrono>
#include <cmath>
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
using example::kRadToDeg;
using example::ret_code_name;
using example::wait_for_joint_state;

namespace {

constexpr int kStateTimeoutMs = 5000;
constexpr int kSettleTimeoutMs = 15000;
constexpr int kSettlePollMs = 50;
constexpr int kSettleStableCycles = 5;
constexpr float kJ1DeltaDeg = 5.0F;
constexpr float kJointSettleTolDeg = 0.8F;

std::mutex g_log_mutex;

struct ArmEndpoint {
    std::string label;
    std::string config_prefix;
    Group command_group = Group::LEFT_ARM;
};

void log_line(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    std::cout << line << '\n';
}

void log_err(const std::string& line) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    std::cerr << line << '\n';
}

void print_joints_deg(const std::string& prefix, const std::vector<float>& q_rad) {
    std::ostringstream oss;
    oss << prefix << " (deg):";
    oss << std::fixed << std::setprecision(2);
    for (size_t i = 0; i < q_rad.size(); ++i) {
        oss << " J" << (i + 1) << "=" << q_rad[i] * kRadToDeg;
    }
    log_line(oss.str());
}

float max_joint_error_deg(const std::vector<float>& a_rad,
                          const std::vector<float>& b_rad) {
    float max_err = 0.0F;
    const size_t n = std::min(a_rad.size(), b_rad.size());
    for (size_t i = 0; i < n; ++i) {
        max_err = std::max(max_err, std::abs(a_rad[i] - b_rad[i]) * kRadToDeg);
    }
    return max_err;
}

bool read_joints(ControlApi& api, Group group, std::vector<float>& q_out) {
    const auto [code, q] = api.get_joint(group);
    if (code != RetCode::SUCCESS || q.empty()) return false;
    q_out = q;
    return true;
}

/** 阻塞 move_joint 只等 ACK；此处轮询反馈直到接近 target 或超时。 */
bool wait_joint_near_target(ControlApi& api, Group group,
                            const std::vector<float>& target_rad,
                            std::vector<float>& reached_rad) {
    const float tol_rad = kJointSettleTolDeg * kDegToRad;
    int stable = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kSettleTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!read_joints(api, group, reached_rad)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kSettlePollMs));
            continue;
        }
        float max_err = 0.0F;
        for (size_t i = 0; i < std::min(reached_rad.size(), target_rad.size());
             ++i) {
            max_err = std::max(max_err, std::abs(reached_rad[i] - target_rad[i]));
        }
        if (max_err <= tol_rad) {
            if (++stable >= kSettleStableCycles) return true;
        } else {
            stable = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kSettlePollMs));
    }
    read_joints(api, group, reached_rad);
    return false;
}

bool enable_arm(ControlApi& api, Group group, const std::string& label) {
    const RetCode en = api.set_group(group, true);
    if (en != RetCode::SUCCESS) {
        log_err("[" + label + "] set_group(" + group_name(group) +
                ", true) -> " + ret_code_name(en));
        return false;
    }
    (void)api.set_group(Group::HEAD, false);
    return true;
}

bool bump_j1_and_back(ControlApi& api, const ArmEndpoint& ep) {
    const Group g = ep.command_group;

    std::vector<float> q_start;
    if (!wait_for_joint_state(api, g, q_start, kStateTimeoutMs)) {
        log_err("[" + ep.label + "] wait joint state timeout");
        return false;
    }

    log_line("[" + ep.label + "] === " + group_name(g) + " ===");
    print_joints_deg("[" + ep.label + "] current (before move)", q_start);

    if (!enable_arm(api, g, ep.label)) {
        log_err("[" + ep.label + "] set_group enable failed");
        return false;
    }

    std::vector<float> q_fwd = q_start;
    q_fwd[0] += kJ1DeltaDeg * kDegToRad;
    print_joints_deg("[" + ep.label + "] target (forward)", q_fwd);

    log_line("[" + ep.label + "] move_joint forward (J1 +" +
             std::to_string(kJ1DeltaDeg) + " deg)");
    const RetCode fwd = api.move_joint(g, q_fwd, false);
    if (fwd != RetCode::SUCCESS) {
        log_err("[" + ep.label + "] move_joint forward -> " +
                ret_code_name(fwd));
        return false;
    }

    std::vector<float> q_after_fwd;
    const bool fwd_settled =
        wait_joint_near_target(api, g, q_fwd, q_after_fwd);
    print_joints_deg("[" + ep.label + "] reached (after forward)",
                     q_after_fwd);
    log_line("[" + ep.label + "] forward max|q-target| = " +
             std::to_string(max_joint_error_deg(q_after_fwd, q_fwd)) +
             " deg" + (fwd_settled ? " (settled)" : " (settle timeout)"));

    print_joints_deg("[" + ep.label + "] target (back)", q_start);
    log_line("[" + ep.label + "] move_joint back");
    const RetCode back = api.move_joint(g, q_start, false);
    if (back != RetCode::SUCCESS) {
        log_err("[" + ep.label + "] move_joint back -> " + ret_code_name(back));
        return false;
    }

    std::vector<float> q_after_back;
    const bool back_settled =
        wait_joint_near_target(api, g, q_start, q_after_back);
    print_joints_deg("[" + ep.label + "] reached (after back)", q_after_back);
    log_line("[" + ep.label + "] back max|q-target| = " +
             std::to_string(max_joint_error_deg(q_after_back, q_start)) +
             " deg" + (back_settled ? " (settled)" : " (settle timeout)"));
    log_line("[" + ep.label + "] back max|q-q_start| = " +
             std::to_string(max_joint_error_deg(q_after_back, q_start)) +
             " deg vs initial");

    return true;
}

void run_arm_task(ArmEndpoint ep, std::atomic<bool>& ok) {
    try {
        log_line("[" + ep.label + "] ControlApi(" + ep.config_prefix + ")");
        ControlApi api(ep.config_prefix);
        if (!bump_j1_and_back(api, ep)) {
            ok.store(false);
            return;
        }
        log_line("[" + ep.label + "] done");
    } catch (const std::exception& e) {
        log_err("[" + ep.label + "] exception: " + e.what());
        ok.store(false);
    }
}

void print_usage(const char* argv0) {
    std::cerr << "Usage: " << argv0
              << " <config_prefix_left> <config_prefix_right>\n"
              << "  Left uses LEFT_ARM, right uses RIGHT_ARM.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 2;
    }

    const ArmEndpoint left_ep{"left_rt", argv[1], Group::LEFT_ARM};
    const ArmEndpoint right_ep{"right_rt", argv[2], Group::RIGHT_ARM};

    log_line("Two ControlApi instances; parallel J1 bump; print current/target/reached.");

    std::atomic<bool> ok{true};
    std::thread t_left(run_arm_task, left_ep, std::ref(ok));
    std::thread t_right(run_arm_task, right_ep, std::ref(ok));
    t_left.join();
    t_right.join();

    if (!ok.load()) {
        log_err("One or both arms failed.");
        return 1;
    }
    log_line("Both arms finished.");
    return 0;
}
