#include "async_dual_arm_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace dual_arm_v2_2_sdk {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float quat_dot(const ToolPose7& a, const ToolPose7& b) {
    return a[3] * b[3] + a[4] * b[4] + a[5] * b[5] + a[6] * b[6];
}

float orientation_error_rad(const ToolPose7& a, const ToolPose7& b) {
    const float dot = std::min(1.0F, std::abs(quat_dot(a, b)));
    return 2.0F * std::acos(dot);
}

float position_error_m(const ToolPose7& a, const ToolPose7& b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void normalize_quat(ToolPose7& p) {
    const float n = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] +
                              p[6] * p[6]);
    if (n < 1e-8F) {
        p[3] = 1.0F;
        p[4] = p[5] = p[6] = 0.0F;
        return;
    }
    for (int i = 3; i < 7; ++i) p[i] /= n;
}

ToolPose7 slerp_tool(const ToolPose7& a, const ToolPose7& b, float s) {
    ToolPose7 out{};
    out[0] = a[0] + (b[0] - a[0]) * s;
    out[1] = a[1] + (b[1] - a[1]) * s;
    out[2] = a[2] + (b[2] - a[2]) * s;

    float q1[4] = {a[3], a[4], a[5], a[6]};
    float q2[4] = {b[3], b[4], b[5], b[6]};
    float dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    if (dot < 0.0F) {
        for (int i = 0; i < 4; ++i) q2[i] = -q2[i];
        dot = -dot;
    }
    dot = std::min(1.0F, std::max(-1.0F, dot));
    if (dot > 0.9995F) {
        for (int i = 0; i < 4; ++i) {
            out[3 + i] = q1[i] + s * (q2[i] - q1[i]);
        }
    } else {
        const float theta = std::acos(dot);
        const float sin_theta = std::sin(theta);
        const float w1 = std::sin((1.0F - s) * theta) / sin_theta;
        const float w2 = std::sin(s * theta) / sin_theta;
        for (int i = 0; i < 4; ++i) {
            out[3 + i] = w1 * q1[i] + w2 * q2[i];
        }
    }
    normalize_quat(out);
    return out;
}

float smooth_step(float u) {
    if (u <= 0.0F) return 0.0F;
    if (u >= 1.0F) return 1.0F;
    return 0.5F - 0.5F * std::cos(kPi * u);
}

std::vector<float> joint_to_vector(const Joint7Rad& q) {
    return std::vector<float>(q.begin(), q.end());
}

Joint7Rad lerp_joint(const Joint7Rad& a, const Joint7Rad& b, float u) {
    Joint7Rad out{};
    const float s = smooth_step(u);
    for (int i = 0; i < 7; ++i) {
        out[i] = a[i] + (b[i] - a[i]) * s;
    }
    return out;
}

}  // namespace

struct AsyncDualArmController::ActiveJointRamp {
    CartesianSide joint_side = CartesianSide::LEFT;
    Joint7Rad start{};
    Joint7Rad end{};
    float duration_s = 1.0F;
    std::chrono::steady_clock::time_point t0{};
    std::atomic<float> progress{0.0F};
};

struct AsyncDualArmController::ActiveLine {
    ToolPose7 start{};
    ToolPose7 end{};
    float duration_s = 1.0F;
    std::chrono::steady_clock::time_point t0{};
    std::atomic<float> progress{0.0F};
};

AsyncDualArmController::AsyncDualArmController(ControlApi& api,
                                               const std::string& config_prefix)
    : api_(api),
      config_prefix_(config_prefix),
      kin_left_(config_prefix, 0),
      kin_right_(config_prefix, 2) {}

AsyncDualArmController::~AsyncDualArmController() { stop(true); }

Kinematics& AsyncDualArmController::task_kinematics() {
    return cartesian_arm_ == CartesianSide::LEFT ? kin_left_ : kin_right_;
}

const Kinematics& AsyncDualArmController::task_kinematics() const {
    return cartesian_arm_ == CartesianSide::LEFT ? kin_left_ : kin_right_;
}

Group AsyncDualArmController::task_group() const {
    return cartesian_arm_ == CartesianSide::LEFT ? Group::LEFT_ARM
                                                 : Group::RIGHT_ARM;
}

Group AsyncDualArmController::joint_group(CartesianSide joint_side) const {
    return joint_side == CartesianSide::LEFT ? Group::LEFT_ARM
                                             : Group::RIGHT_ARM;
}

CartesianSide AsyncDualArmController::expected_joint_side() const {
    return cartesian_arm_ == CartesianSide::LEFT ? CartesianSide::RIGHT
                                                 : CartesianSide::LEFT;
}

MotionRetCode AsyncDualArmController::validate_joint_side(
    CartesianSide joint_side) const {
    if (joint_side != expected_joint_side()) {
        return MotionRetCode::INVALID_ARGUMENT;
    }
    return MotionRetCode::SUCCESS;
}

void AsyncDualArmController::send_arm_joints(Group group, const Joint7Rad& q) {
    const auto joints = joint_to_vector(q);
    if (group == Group::LEFT_ARM) {
        api_.set_left_joint_target(joints);
    } else if (group == Group::RIGHT_ARM) {
        api_.set_right_joint_target(joints);
    }
}

bool AsyncDualArmController::fetch_task_seed(Joint7Rad& seed) const {
    const auto [code, joints] = api_.get_joint(task_group());
    if (code != RetCode::SUCCESS || joints.size() != 7) return false;
    for (int i = 0; i < 7; ++i) seed[i] = joints[i];
    return true;
}

bool AsyncDualArmController::fetch_tool_start(ToolPose7& start) const {
    const auto [code, pose] = api_.get_tool_pose(task_group());
    if (code != RetCode::SUCCESS || pose.size() != 7) return false;
    for (int i = 0; i < 7; ++i) start[i] = pose[i];
    return true;
}

MotionRetCode AsyncDualArmController::prescan_tool_line(const ToolPose7& start,
                                                        const ToolPose7& end) const {
    if (!options_.singularity.reject_entire_line) return MotionRetCode::SUCCESS;

    const auto& policy = options_.singularity;
    float ds = 1.0F;
    Joint7Rad seed{};
    if (!fetch_task_seed(seed)) return MotionRetCode::NO_STATE;

    float s = 0.0F;
    while (s <= 1.0F + 1e-6F) {
        const float u = smooth_step(s);
        ToolPose7 via = slerp_tool(start, end, u);
        Joint7Rad q_out{};
        const MotionRetCode ik = task_kinematics().inverse_tool(
            via, seed, q_out, policy);
        if (ik != MotionRetCode::SUCCESS) {
            if (ds > policy.line_ds_min) {
                s = std::max(0.0F, s - ds);
                ds *= 0.5F;
                continue;
            }
            return ik == MotionRetCode::IK_NEAR_SINGULAR ? MotionRetCode::PATH_BLOCKED
                                                         : ik;
        }
        seed = q_out;
        s += ds;
    }
    return MotionRetCode::SUCCESS;
}

MotionRetCode AsyncDualArmController::start(const AsyncStreamOptions& options) {
    if (planner_running_.load()) return MotionRetCode::CONTROLLER_BUSY;
    if (api_.is_dual_arm_streaming() || api_.is_dual_arm_tool_line_auto_flush()) {
        return MotionRetCode::CONTROLLER_BUSY;
    }

    options_ = options;
    cartesian_arm_ = options.cartesian_arm;
    last_error_.store(MotionRetCode::SUCCESS);
    state_.store(AsyncMotionState::IDLE);

    if (!fetch_task_seed(task_q_last_good_)) {
        return MotionRetCode::NO_STATE;
    }
    task_q_cmd_ = task_q_last_good_;

    const int hz = std::max(1, options_.coalesce_hz);
    if (api_.start_dual_arm_streaming(true, 0, 999, hz) != RetCode::SUCCESS) {
        return MotionRetCode::NO_STATE;
    }

    hold_both_from_feedback();
    planner_running_.store(true);
    state_.store(AsyncMotionState::STREAMING);
    planner_thread_ = std::thread(&AsyncDualArmController::planner_loop, this);
    return MotionRetCode::SUCCESS;
}

void AsyncDualArmController::hold_both_from_feedback() {
    Joint7Rad left{};
    Joint7Rad right{};
    if (fetch_task_seed(task_q_cmd_)) {
        /* seed ok */
    }
    const auto [lc, lj] = api_.get_joint(Group::LEFT_ARM);
    const auto [rc, rj] = api_.get_joint(Group::RIGHT_ARM);
    if (lc == RetCode::SUCCESS && lj.size() == 7) {
        for (int i = 0; i < 7; ++i) left[i] = lj[i];
    }
    if (rc == RetCode::SUCCESS && rj.size() == 7) {
        for (int i = 0; i < 7; ++i) right[i] = rj[i];
    }
    send_arm_joints(Group::LEFT_ARM, left);
    send_arm_joints(Group::RIGHT_ARM, right);
}

void AsyncDualArmController::stop(bool hold_last_targets) {
    planner_running_.store(false);
    if (planner_thread_.joinable()) planner_thread_.join();
    api_.stop_dual_arm_streaming();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_line_.reset();
        active_joint_ramp_.reset();
    }
    if (hold_last_targets) {
        hold_both_from_feedback();
    }
    state_.store(AsyncMotionState::IDLE);
}

AsyncMotionState AsyncDualArmController::state() const { return state_.load(); }

MotionRetCode AsyncDualArmController::last_error() const {
    return last_error_.load();
}

MotionRetCode AsyncDualArmController::set_joint_target(CartesianSide joint_side,
                                                       const Joint7Rad& target_rad,
                                                       float duration_s) {
    if (!planner_running_.load()) return MotionRetCode::NOT_INITIALIZED;
    const MotionRetCode side_ok = validate_joint_side(joint_side);
    if (side_ok != MotionRetCode::SUCCESS) return side_ok;
    if (duration_s <= 0.0F) return MotionRetCode::INVALID_ARGUMENT;

    Joint7Rad start{};
    const auto [code, joints] = api_.get_joint(joint_group(joint_side));
    if (code != RetCode::SUCCESS || joints.size() != 7) {
        return MotionRetCode::NO_STATE;
    }
    for (int i = 0; i < 7; ++i) start[i] = joints[i];

    auto ramp = std::make_shared<ActiveJointRamp>();
    ramp->joint_side = joint_side;
    ramp->start = start;
    ramp->end = target_rad;
    ramp->duration_s = duration_s;
    ramp->t0 = std::chrono::steady_clock::now();
    ramp->progress.store(0.0F);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_joint_ramp_ = ramp;
    }
    return MotionRetCode::SUCCESS;
}

MotionRetCode AsyncDualArmController::hold_joint_arm(CartesianSide joint_side) {
    if (!planner_running_.load()) return MotionRetCode::NOT_INITIALIZED;
    const MotionRetCode side_ok = validate_joint_side(joint_side);
    if (side_ok != MotionRetCode::SUCCESS) return side_ok;

    const auto [code, joints] = api_.get_joint(joint_group(joint_side));
    if (code != RetCode::SUCCESS || joints.size() != 7) {
        return MotionRetCode::NO_STATE;
    }
    Joint7Rad q{};
    for (int i = 0; i < 7; ++i) q[i] = joints[i];
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_joint_ramp_.reset();
    }
    send_arm_joints(joint_group(joint_side), q);
    return MotionRetCode::SUCCESS;
}

MotionRetCode AsyncDualArmController::move_tool_line(const ToolLineSpec& spec) {
    if (!planner_running_.load()) return MotionRetCode::NOT_INITIALIZED;

    ToolPose7 start = spec.start;
    if (spec.start_is_current) {
        if (!fetch_tool_start(start)) return MotionRetCode::NO_STATE;
    }
    ToolPose7 end = spec.end;
    normalize_quat(start);
    normalize_quat(end);

    const MotionRetCode scan =
        prescan_tool_line(start, end);
    if (scan != MotionRetCode::SUCCESS) {
        last_error_.store(scan);
        state_.store(AsyncMotionState::ERROR);
        return scan;
    }

    const float dist = position_error_m(start, end);
    const float ori = orientation_error_rad(start, end);
    const float speed = std::max(0.05F, 0.2F * spec.speed_ratio / 100.0F);
    const float duration =
        spec.acc_time_s * 2.0F + dist / speed + ori * spec.acc_time_s * 0.25F;

    auto line = std::make_shared<ActiveLine>();
    line->start = start;
    line->end = end;
    line->duration_s = std::max(0.5F, duration);
    line->t0 = std::chrono::steady_clock::now();
    line->progress.store(0.0F);
    task_goal_ = end;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_line_ = line;
    }
    state_.store(AsyncMotionState::TASK_MOVING);
    return MotionRetCode::SUCCESS;
}

MotionRetCode AsyncDualArmController::cancel_cartesian_motion() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_line_.reset();
    state_.store(planner_running_.load() ? AsyncMotionState::STREAMING
                                         : AsyncMotionState::IDLE);
    return MotionRetCode::SUCCESS;
}

float AsyncDualArmController::cartesian_progress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_line_) return 1.0F;
    return active_line_->progress.load();
}

float AsyncDualArmController::joint_move_progress(
    CartesianSide joint_side) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_joint_ramp_ || active_joint_ramp_->joint_side != joint_side) {
        return 1.0F;
    }
    return active_joint_ramp_->progress.load();
}

void AsyncDualArmController::apply_joint_ramp_step() {
    std::shared_ptr<ActiveJointRamp> ramp;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ramp = active_joint_ramp_;
    }
    if (!ramp) return;

    const auto now = std::chrono::steady_clock::now();
    const float elapsed =
        std::chrono::duration<float>(now - ramp->t0).count();
    const float u = std::min(1.0F, elapsed / ramp->duration_s);
    ramp->progress.store(u);

    const Joint7Rad q_cmd = lerp_joint(ramp->start, ramp->end, u);
    send_arm_joints(joint_group(ramp->joint_side), q_cmd);

    if (u >= 1.0F) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_joint_ramp_ == ramp) active_joint_ramp_.reset();
    }
}

void AsyncDualArmController::apply_task_step() {
    std::shared_ptr<ActiveLine> line;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        line = active_line_;
    }
    if (!line) return;

    const auto now = std::chrono::steady_clock::now();
    const float elapsed =
        std::chrono::duration<float>(now - line->t0).count();
    const float u = std::min(1.0F, elapsed / line->duration_s);
    line->progress.store(u);

    const ToolPose7 via = slerp_tool(line->start, line->end, smooth_step(u));
    Joint7Rad seed = task_q_cmd_;
    if (!fetch_task_seed(seed)) seed = task_q_last_good_;

    Joint7Rad q_out{};
    const MotionRetCode ik = task_kinematics().inverse_tool(
        via, seed, q_out, options_.singularity);
    if (ik != MotionRetCode::SUCCESS) {
        last_error_.store(ik);
        state_.store(AsyncMotionState::TASK_HOLD);
        send_arm_joints(task_group(), task_q_last_good_);
        std::lock_guard<std::mutex> lock(mutex_);
        active_line_.reset();
        return;
    }

    task_q_cmd_ = q_out;
    task_q_last_good_ = q_out;
    send_arm_joints(task_group(), q_out);

    if (u >= 1.0F) {
        std::lock_guard<std::mutex> lock(mutex_);
        active_line_.reset();
        state_.store(AsyncMotionState::STREAMING);
    }
}

void AsyncDualArmController::planner_loop() {
    const int period_ms =
        std::max(1, 1000 / std::max(1, options_.coalesce_hz));
    auto next = std::chrono::steady_clock::now();
    while (planner_running_.load()) {
        apply_joint_ramp_step();
        apply_task_step();
        next += std::chrono::milliseconds(period_ms);
        std::this_thread::sleep_until(next);
    }
}

MotionRetCode AsyncDualArmController::poll_reach(float* pos_err_m,
                                                 float* ori_err_rad) const {
    const auto [code, pose_vec] = api_.get_tool_pose(task_group());
    if (code != RetCode::SUCCESS || pose_vec.size() != 7) {
        return MotionRetCode::NO_STATE;
    }
    ToolPose7 current{};
    for (int i = 0; i < 7; ++i) current[i] = pose_vec[i];

    const float pe = position_error_m(current, task_goal_);
    const float oe = orientation_error_rad(current, task_goal_);
    if (pos_err_m) *pos_err_m = pe;
    if (ori_err_rad) *ori_err_rad = oe;

    if (pe <= options_.tool_pos_tolerance_m &&
        oe <= options_.tool_ori_tolerance_rad) {
        return MotionRetCode::SUCCESS;
    }
    return MotionRetCode::INVALID_ARGUMENT;
}

}  // namespace dual_arm_v2_2_sdk
