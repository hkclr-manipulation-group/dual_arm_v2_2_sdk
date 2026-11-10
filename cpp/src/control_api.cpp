#include "control_api.h"
#include "gripper_config.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/time_utils.h"

namespace dual_arm_v2_2_sdk {
namespace {

// PlannerState on the wire is about 7 KB. A 4096-byte receive buffer cuts off
// system_state and the diagnostic flags; the failed sscanf then leaves stack
// garbage that motion_fault() treats as a controller fault.
constexpr int kBufferSize = 16384;
constexpr int kAckTimeoutMs = 200;
constexpr int kTaskAckTimeoutMs = 10000;
constexpr int kTaskResendTimeoutMs = 1000;
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0F;
constexpr float kRadToDeg = 180.0F / static_cast<float>(M_PI);
constexpr float kMinimumProfileTime = 0.01F;
constexpr float kTaskPosEpsM = 1e-5F;
constexpr float kTaskOriEpsRad = 1e-4F;
constexpr float kTaskOriEpsDeg = kTaskOriEpsRad * kRadToDeg;
// Feedback "arrived" window. Packet ACK is not arrival: quintic keeps running
// after the planner echoes the sequence.
constexpr float kJointReachTolRad = 0.01F * kDegToRad;
constexpr float kToolPosReachTolM = 0.0005F;
constexpr float kToolRotReachTolRad = 0.01F * kDegToRad;
constexpr int kReachStableCycles = 3;
constexpr int kReachPollMs = 20;
constexpr int kMotionReachTimeoutMs = 20000;
constexpr uint32_t kMotionFaultMask =
    DiagnosticFlags::kFaultPosHardLimitReached |
    DiagnosticFlags::kFaultVelHardLimitReached |
    DiagnosticFlags::kFaultTorHardLimitReached |
    DiagnosticFlags::kFaultPosTrackingFailed |
    DiagnosticFlags::kFaultVelTrackingFailed |
    DiagnosticFlags::kFaultTorTrackingFailed;

float curve_fit_time(uint16_t radio) {
    // 0..100 maps to 10 ms..1.01 s; a larger value produces a smoother fit.
    return kMinimumProfileTime + static_cast<float>(radio) / 100.0F;
}

float filter_cutoff_frequency(uint16_t radio) {
    // Logarithmic mapping gives perceptually smoother adjustment across 20..1 Hz.
    constexpr float kMaximumCutoffHz = 20.0F;
    constexpr float kMinimumCutoffHz = 1.0F;
    const float normalized = static_cast<float>(radio) / 999.0F;
    return kMaximumCutoffHz *
           std::pow(kMinimumCutoffHz / kMaximumCutoffHz, normalized);
}

constexpr int kToolPoseSize = 7;

struct EulerRpy {
    double roll;
    double pitch;
    double yaw;
};

struct Quat {
    double w;
    double x;
    double y;
    double z;
};

// q_cur^{-1} * q_tar. Inverse of RoboticsBase::toQuat (Rx then Ry then Rz)
// when the target was built as q_cur * toQuat(Δ).
Quat get_relative_quat(const float cur[4], const float tar[4]) {
    const double cw = cur[0];
    const double cx = -cur[1];
    const double cy = -cur[2];
    const double cz = -cur[3];
    const double tw = tar[0];
    const double tx = tar[1];
    const double ty = tar[2];
    const double tz = tar[3];

    Quat q;
    q.w = cw * tw - cx * tx - cy * ty - cz * tz;
    q.x = cw * tx + cx * tw + cy * tz - cz * ty;
    q.y = cw * ty - cx * tz + cy * tw + cz * tx;
    q.z = cw * tz + cx * ty - cy * tx + cz * tw;

    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 1e-6) {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }
    return q;
}

float tool_rotation_error_rad(const float current_quat[4],
                              const float target_quat[4]) {
    const Quat relative = get_relative_quat(current_quat, target_quat);
    const double w = std::min(1.0, std::abs(relative.w));
    return static_cast<float>(2.0 * std::acos(w));
}

// Matches Eigen R.eulerAngles(0, 1, 2), the inverse of RoboticsBase::toQuat.
EulerRpy quaternion_to_eigen_euler_xyz(double w, double x, double y, double z) {
    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - w * z);
    const double r02 = 2.0 * (x * z + w * y);
    const double r12 = 2.0 * (y * z - w * x);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    const double pitch = std::asin(std::max(-1.0, std::min(1.0, r02)));
    const double cy = std::cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (cy > 1e-4) {
        roll = std::atan2(-r12, r22);
        yaw = std::atan2(-r01, r00);
    } else {
        const double r10 = 2.0 * (x * y + w * z);
        const double r11 = 1.0 - 2.0 * (x * x + z * z);
        roll = r02 > 0.0 ? std::atan2(r10, r11) : -std::atan2(r10, r11);
    }
    return {roll, pitch, yaw};
}

// kEnd MoveL: R_target = R0 * toR(rpy_cmd) => rpy_cmd = eulerXYZ(R0^T * R_target).
void fill_task_line_delta(float* task_cmd_deg,
                          const float current_tool_pose[kToolPoseSize],
                          const float target_tool_pose[kToolPoseSize]) {
    task_cmd_deg[0] = target_tool_pose[0] - current_tool_pose[0];
    task_cmd_deg[1] = target_tool_pose[1] - current_tool_pose[1];
    task_cmd_deg[2] = target_tool_pose[2] - current_tool_pose[2];

    const Quat q_rel =
        get_relative_quat(&current_tool_pose[3], &target_tool_pose[3]);
    const EulerRpy rpy_inc =
        quaternion_to_eigen_euler_xyz(q_rel.w, q_rel.x, q_rel.y, q_rel.z);

    // TaskCmdDeg[3..5] are degrees; TaskCmd gets radians via sync_task_cmd_from_deg.
    task_cmd_deg[3] = static_cast<float>(rpy_inc.roll * kRadToDeg);
    task_cmd_deg[4] = static_cast<float>(rpy_inc.pitch * kRadToDeg);
    task_cmd_deg[5] = static_cast<float>(rpy_inc.yaw * kRadToDeg);
}

float clamp_speed_ratio(uint16_t speed_ratio) {
    return std::clamp(static_cast<float>(speed_ratio) / 100.0F, 0.0F, 1.0F);
}

const char* tool_arm_label(int arm) {
    if (arm == 0) return "L";
    if (arm == 2) return "R";
    return "?";
}

}  // namespace

struct ControlApi::Impl {
    YAML::Node config;
    PanelCommand command{};
    PlannerState state{};
    std::unique_ptr<CuarmUdp<PlannerState, PanelCommand>> udp;
    std::thread receive_thread;
    std::atomic<bool> shutdown{false};
    std::atomic<bool> received_state{false};
    std::atomic<uint64_t> state_generation{0};
    mutable std::mutex state_mutex;
    std::mutex command_mutex;
    std::vector<int> joint_sizes;
    std::vector<std::string> gripper_names;
    std::vector<std::vector<float>> max_acceleration;
    uint8_t previous_trajectory_mode = 255;
    bool previous_follow = false;
    bool joint_target_cache_valid = false;
    float joint_target_cache[MAX_ARM_SIZE][MAX_JOINT_SIZE]{};
    std::array<bool, MAX_ARM_SIZE> arm_target_dirty{};
    std::thread streaming_thread;
    std::atomic<bool> streaming_active{false};
    bool streaming_follow = true;
    uint8_t streaming_trajectory_mode = 2;
    uint16_t streaming_radio = 999;
    int streaming_hz = 100;
    float tool_target_cache[MAX_ARM_SIZE][kToolPoseSize]{};
    bool tool_target_cache_valid = false;
    std::array<bool, MAX_ARM_SIZE> tool_target_dirty{};
    float movel_target_tool_pose_[MAX_ARM_SIZE][kToolPoseSize]{};
    std::array<bool, MAX_ARM_SIZE> movel_target_valid_{};
    std::thread tool_streaming_thread;
    std::atomic<bool> tool_streaming_active{false};
    std::condition_variable tool_flush_cv;
    bool tool_flush_pending = false;
    int tool_coalesce_ms = 10;
    uint16_t tool_stream_speed_ratio = 50;
    float tool_stream_acc_time = 0.0F;
    bool tool_stream_profile_initialized = false;
    float receive_dt_us_ = 9000.0F;
    MotionControl last_motion_type_ = MotionControl::kJoint;
    ControlType last_target_type_ = ControlType::kPosition;
    ControlType last_actuator_mode_ = ControlType::kPosition;

    static bool is_streaming_arm(int arm) { return arm == 0 || arm == 2; }

    explicit Impl(const std::string& prefix) {
        config = loadYamlConfig(prefix + "/config.yaml");
        initialize_command();

        const auto panel = config["panel"];
        const auto rt = config["rt_control"];
        udp = std::make_unique<CuarmUdp<PlannerState, PanelCommand>>(
            panel["ip"].as<std::string>(), panel["port"].as<int>(),
            rt["ip"].as<std::string>(), rt["port"].as<int>(),
            &CuarmMessageHandler::unpack_planner_state,
            &CuarmMessageHandler::pack_panel_command, kBufferSize);

        receive_thread = std::thread(&Impl::receive_loop, this);
    }

    ~Impl() { stop_all_streaming_loops(); }

    void stop_joint_streaming_loop() {
        streaming_active.store(false);
        if (streaming_thread.joinable()) streaming_thread.join();
    }

    void stop_tool_streaming_loop() {
        tool_streaming_active.store(false);
        if (tool_streaming_thread.joinable()) tool_streaming_thread.join();
    }

    void stop_all_streaming_loops() {
        stop_joint_streaming_loop();
        stop_tool_streaming_loop();
        shutdown.store(true);
        if (receive_thread.joinable()) receive_thread.join();
        if (udp) udp->close();
    }

    void initialize_command() {
        const auto arms = config["robot"]["arm"];
        command.connection_state = ConnectionState::kRemote;
        // The planner starts with received_sequence_id == 0. Starting at 1
        // prevents an unsolicited startup state from being mistaken for the
        // acknowledgement of our first command.
        command.sequence_id = 1;
        command.need_setting_update = true;
        command.simulation = config["rt_control"]["hardware_simulation"].as<bool>();
        command.motion_type = MotionControl::kJoint;
        command.target_type = ControlType::kPosition;
        command.actuator_mode = ControlType::kPosition;
        command.interpolation_type = InterpolationMethod::kQuintic;
        command.interpolation_speed_ratio = 1.0F;
        command.InterpolationAccTime = config["panel"]["general"]["acc_time"].as<float>();
        command.InterpolationConstVelTime = config["panel"]["general"]["vel_time"].as<float>();
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.ArmSize = static_cast<uint8_t>(arms.size());

        for (int arm = 0; arm < command.ArmSize; ++arm) {
            const auto node = arms[arm];
            const int count = node["joint_size"].as<int>();
            joint_sizes.push_back(count);
            max_acceleration.emplace_back(count, 0.0F);
            command.JointSize[arm] = static_cast<uint8_t>(count);
            const std::string robot_name = node["name"].as<std::string>() + "-" + node["type"].as<std::string>();
            std::strncpy(command.RobotName[arm], robot_name.c_str(), MAX_ROBOTNAME_SIZE - 1);
            for (int joint = 0; joint < count; ++joint) {
                command.enable_joint[arm][joint] = node["enable_joint"][joint].as<bool>();
                command.arm_soft_limit_position[arm][joint][0] =
                    node["safety"]["joint_soft_limit"]["position"][0][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                command.arm_soft_limit_position[arm][joint][1] =
                    node["safety"]["joint_soft_limit"]["position"][1][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                const float max_velocity = node["safety"]["joint_soft_limit"]["velocity"][joint].as<float>() * static_cast<float>(M_PI / 180.0);
                command.arm_soft_limit_velocity[arm][joint][0] = -max_velocity;
                command.arm_soft_limit_velocity[arm][joint][1] = max_velocity;
                if (node["safety"]["joint_soft_limit"]["acceleration"])
                    max_acceleration[arm][joint] = node["safety"]["joint_soft_limit"]["acceleration"][joint].as<float>() * static_cast<float>(M_PI / 180.0);
            }
        }

        const auto grippers = config["robot"]["gripper"];
        command.GripperSize = grippers ? static_cast<uint8_t>(grippers.size()) : 0;
        for (int gripper = 0; gripper < command.GripperSize; ++gripper)
        {
            gripper_names.push_back(grippers[gripper]["name"].as<std::string>());
            command.GripperJointSize[gripper] =
                grippers[gripper]["joint_size"].as<uint8_t>();
            command.GripperSpeed[gripper] = CUARM_GRIPPER_DEFAULT_SPEED;
            command.GripperForce[gripper] = CUARM_GRIPPER_DEFAULT_FORCE;
        }

        receive_dt_us_ = config["panel"]["dt_receive"].as<float>() * 1000000.0F;
        last_motion_type_ = static_cast<MotionControl>(command.motion_type);
        last_target_type_ = command.target_type;
        last_actuator_mode_ = command.actuator_mode;
    }

    void receive_loop() {
        while (!shutdown.load()) {
            PlannerState incoming{};
            if (udp->receive(&incoming, 10000)) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex);
                    state = incoming;
                }
                received_state.store(true);
                state_generation.fetch_add(1);
            }
        }
    }

    std::vector<int> group_indices(Group group) const {
        switch (group) {
            case Group::ALL: return {0, 1, 2};
            case Group::LEFT_ARM: return {0};
            case Group::RIGHT_ARM: return {2};
            case Group::HEAD: return {1};
            case Group::ELEVATOR: return {};
        }
        return {};
    }

    std::vector<int> tool_arm_indices(Group group) const {
        switch (group) {
            case Group::ALL: return {0, 2};
            case Group::LEFT_ARM: return {0};
            case Group::RIGHT_ARM: return {2};
            default: return {};
        }
    }

    size_t expected_tool_pose_count(Group group) const {
        return tool_arm_indices(group).size() * static_cast<size_t>(kToolPoseSize);
    }

    size_t expected_joint_count(Group group) const {
        size_t result = 0;
        for (int arm : group_indices(group)) {
            if (arm < 0 || arm >= static_cast<int>(joint_sizes.size())) return 0;
            result += static_cast<size_t>(joint_sizes[arm]);
        }
        return result;
    }

    int gripper_index(Group group) const {
        if (command.GripperSize == 0) return -1;

        const char* target_name = nullptr;
        if (group == Group::LEFT_ARM) target_name = "gripperL";
        else if (group == Group::RIGHT_ARM) target_name = "gripperR";
        else return -1;

        for (int i = 0; i < static_cast<int>(gripper_names.size()); ++i) {
            if (gripper_names[i] == target_name) return i;
        }

        if (command.GripperSize == 1) {
            if (group == Group::LEFT_ARM && CUARM_GRIPPER_ARM_INDEX == 0) return 0;
            if (group == Group::RIGHT_ARM && CUARM_GRIPPER_ARM_INDEX == 2) return 0;
        }
        return -1;
    }

    PlannerState state_snapshot() const {
        std::lock_guard<std::mutex> lock(state_mutex);
        return state;
    }

    void synchronize_sequence_with_planner() {
        if (!received_state.load()) return;
        const uint32_t received_sequence = state_snapshot().received_sequence_id;
        if (command.sequence_id <= received_sequence)
            command.sequence_id = received_sequence + 1;
    }

    bool send_once() {
        try {
            command.send_timestamp = get_time_now();
            udp->send(&command);
            ++command.sequence_id;
            return true;
        } catch (...) {
            return false;
        }
    }

    bool wait_for_ack(uint32_t sequence, uint64_t generation_before_send,
                      int timeout_ms = kAckTimeoutMs) const {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            if (state_generation.load() > generation_before_send &&
                state_snapshot().received_sequence_id >= sequence)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    RetCode motion_fault(const std::vector<int>& arm_indices) const {
        const PlannerState snapshot = state_snapshot();
        if (snapshot.system_state == SystemState::kError ||
            (snapshot.system_diagnostic_flags & kMotionFaultMask) != 0)
            return RetCode::CONTROLLER_ERROR;
        for (int arm : arm_indices) {
            if (arm < 0 || arm >= static_cast<int>(joint_sizes.size())) continue;
            for (int joint = 0; joint < joint_sizes[arm]; ++joint) {
                if ((snapshot.arm_joint_diagnostic_flags[arm][joint] &
                     kMotionFaultMask) != 0)
                    return RetCode::CONTROLLER_ERROR;
            }
        }
        return RetCode::SUCCESS;
    }

    bool joints_reached(const std::vector<int>& arm_indices,
                        const std::vector<float>& joints) const {
        const PlannerState snapshot = state_snapshot();
        size_t source = 0;
        for (int arm : arm_indices) {
            for (int joint = 0; joint < joint_sizes[arm]; ++joint) {
                if (source >= joints.size()) return false;
                if (std::abs(snapshot.JointPos[arm][joint] - joints[source]) >
                    kJointReachTolRad)
                    return false;
                ++source;
            }
        }
        return source == joints.size();
    }

    bool tool_pose_reached(const std::vector<int>& arm_indices,
                           const std::vector<float>& target_tool_pose) const {
        const PlannerState snapshot = state_snapshot();
        size_t source = 0;
        for (int arm : arm_indices) {
            if (source + static_cast<size_t>(kToolPoseSize) > target_tool_pose.size())
                return false;
            const float* current = snapshot.ToolPose[arm];
            const float* target = target_tool_pose.data() + source;
            const float dx = current[0] - target[0];
            const float dy = current[1] - target[1];
            const float dz = current[2] - target[2];
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > kToolPosReachTolM)
                return false;
            if (tool_rotation_error_rad(&current[3], &target[3]) > kToolRotReachTolRad)
                return false;
            source += static_cast<size_t>(kToolPoseSize);
        }
        return true;
    }

    template <typename Reached>
    RetCode wait_until_reached(const std::vector<int>& arm_indices,
                               Reached reached) const {
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(kMotionReachTimeoutMs);
        int stable = 0;
        while (std::chrono::steady_clock::now() < deadline) {
            const RetCode fault = motion_fault(arm_indices);
            if (fault != RetCode::SUCCESS) return fault;
            if (reached()) {
                if (++stable >= kReachStableCycles) return RetCode::SUCCESS;
            } else {
                stable = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kReachPollMs));
        }
        return RetCode::TIMEOUT;
    }

    RetCode send_with_ack(int timeout_ms = kAckTimeoutMs) {
        // rt_control can outlive this SDK object/process. Continue after the
        // sequence number it already accepted, rather than restarting at 1
        // and having every packet rejected as stale.
        synchronize_sequence_with_planner();
        const uint32_t sequence = command.sequence_id;
        const uint64_t generation_before_send = state_generation.load();
        if (!send_once()) return RetCode::SEND_FAILED;
        return wait_for_ack(sequence, generation_before_send, timeout_ms)
                   ? RetCode::SUCCESS
                   : RetCode::TIMEOUT;
    }

    void sync_task_cmd_from_deg() {
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            for (int axis = 0; axis < 6; ++axis) {
                command.TaskCmd[arm][axis] = command.TaskCmdDeg[arm][axis];
                if (axis > 2) command.TaskCmd[arm][axis] *= kDegToRad;
            }
        }
    }

    void log_movel_udp_payload() const {
        if (static_cast<MotionControl>(command.motion_type) !=
            MotionControl::kTaskLine) {
            return;
        }
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "[ControlApi] UDP MoveL send seq(next)="
                  << command.sequence_id << '\n';
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (!is_tool_motion_arm(arm) || !movel_target_valid_[arm]) continue;
            const float* t = movel_target_tool_pose_[arm];
            const float* deg = command.TaskCmdDeg[arm];
            const float* rad = command.TaskCmd[arm];
            std::cout << "  arm[" << arm << "] " << tool_arm_label(arm)
                      << " target_tool_pose (m,qw,qx,qy,qz): " << t[0] << ", "
                      << t[1] << ", " << t[2] << ", " << t[3] << ", " << t[4]
                      << ", " << t[5] << ", " << t[6] << '\n';
            std::cout << std::setprecision(3);
            std::cout << "    TaskCmdDeg [dx,dy,dz m | r,p,y deg]: " << deg[0]
                      << ", " << deg[1] << ", " << deg[2] << ", " << deg[3]
                      << ", " << deg[4] << ", " << deg[5] << '\n';
            std::cout << std::setprecision(6);
            std::cout << "    TaskCmd    [dx,dy,dz m | r,p,y rad]: " << rad[0]
                      << ", " << rad[1] << ", " << rad[2] << ", " << rad[3]
                      << ", " << rad[4] << ", " << rad[5] << '\n';
        }
        std::cout << std::flush;
    }

    bool is_latest_target_received() const {
        return state_snapshot().received_sequence_id == command.sequence_id - 1;
    }

    RetCode send_task_with_ack() {
        if (command.actuator_mode != last_actuator_mode_ ||
            static_cast<MotionControl>(command.motion_type) !=
                last_motion_type_ ||
            command.reset_interpolation || command.reset_control_mem) {
            command.need_setting_update = true;
        }

        const int poll_ms =
            std::max(1, static_cast<int>(receive_dt_us_ * 2.0F / 1000.0F));
        int elapsed_ms = 0;
        bool need_send = true;

        while (elapsed_ms < kTaskAckTimeoutMs && need_send) {
            synchronize_sequence_with_planner();
            sync_task_cmd_from_deg();
            log_movel_udp_payload();
            if (!send_once()) return RetCode::SEND_FAILED;

            int wait_ms = 0;
            while (wait_ms < kTaskResendTimeoutMs) {
                const PlannerState snapshot = state_snapshot();
                if (command.need_setting_update) {
                    if (snapshot.received_sequence_id == command.sequence_id - 1) {
                        command.need_setting_update = false;
                        command.reset_interpolation = false;
                        command.reset_control_mem = false;
                        break;
                    }
                } else if (snapshot.setting_update_finished &&
                           snapshot.received_sequence_id ==
                               command.sequence_id - 1) {
                    need_send = false;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
                wait_ms += poll_ms;
            }

            last_motion_type_ = static_cast<MotionControl>(command.motion_type);
            last_target_type_ = command.target_type;
            last_actuator_mode_ = command.actuator_mode;
            elapsed_ms += wait_ms;
        }

        return need_send ? RetCode::TIMEOUT : RetCode::SUCCESS;
    }

    void hold_all_groups_at_current_position() {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = snapshot.JointPos[arm][joint];
        for (int gripper = 0;
             gripper < command.GripperSize && gripper < snapshot.GripperSize;
             ++gripper)
            for (int joint = 0;
                 joint < command.GripperJointSize[gripper]; ++joint)
                command.GripperJointCmd[gripper][joint] =
                    snapshot.GripperJointPos[gripper][joint];
    }

    RetCode apply_settings() {
        command.need_setting_update = true;
        RetCode result = send_with_ack();
        if (result != RetCode::SUCCESS) return result;

        // The settings packet makes rt_control refresh feedback without
        // applying a motion command. Use that feedback as the first enabled
        // position target, so enabling can never fall through to zero-filled
        // JointCmd values.
        hold_all_groups_at_current_position();
        command.need_setting_update = false;
        result = send_with_ack();
        if (result == RetCode::SUCCESS) sync_joint_target_cache_from_command();
        return result;
    }

    RetCode ensure_initialized() {
        return command.need_setting_update ? apply_settings()
                                           : RetCode::SUCCESS;
    }

    void hold_unselected_groups(const std::vector<int>& selected) {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (std::find(selected.begin(), selected.end(), arm) != selected.end()) continue;
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = snapshot.JointPos[arm][joint];
        }
    }

    void clear_arm_target_dirty() { arm_target_dirty.fill(false); }

    void sync_joint_target_cache_from_command() {
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                joint_target_cache[arm][joint] = command.JointCmd[arm][joint];
        joint_target_cache_valid = true;
        clear_arm_target_dirty();
    }

    void sync_joint_target_cache_from_state() {
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                joint_target_cache[arm][joint] = snapshot.JointPos[arm][joint];
        joint_target_cache_valid = true;
        clear_arm_target_dirty();
    }

    bool ensure_joint_target_cache() {
        if (joint_target_cache_valid) return true;
        if (!received_state.load()) return false;
        sync_joint_target_cache_from_state();
        return true;
    }

    void hold_unflushed_groups_in_cache(bool sticky_left_right) {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (sticky_left_right && is_streaming_arm(arm)) continue;
            if (arm_target_dirty[arm]) continue;
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                joint_target_cache[arm][joint] = snapshot.JointPos[arm][joint];
        }
    }

    RetCode set_joint_target_locked(Group group, const std::vector<float>& joints) {
        const auto indices = group_indices(group);
        if (indices.empty() || joints.size() != expected_joint_count(group))
            return RetCode::CONTROLLER_ERROR;
        if (!ensure_joint_target_cache()) return RetCode::RECEIVE_FAILED;

        size_t source = 0;
        for (int arm : indices) {
            arm_target_dirty[arm] = true;
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                joint_target_cache[arm][joint] = joints[source++];
        }
        return RetCode::SUCCESS;
    }

    void streaming_loop() {
        const int period_ms = streaming_hz > 0 ? 1000 / streaming_hz : 10;
        auto next_wakeup = std::chrono::steady_clock::now();
        while (streaming_active.load()) {
            {
                std::lock_guard<std::mutex> lock(command_mutex);
                send_cached_joint_command(streaming_follow, streaming_trajectory_mode,
                                          streaming_radio, true);
            }
            next_wakeup += std::chrono::milliseconds(period_ms);
            std::this_thread::sleep_until(next_wakeup);
        }
    }

    void apply_joint_motion_profile(bool follow, uint8_t trajectory_mode,
                                    uint16_t radio, uint16_t speed_ratio = 0) {
        command.motion_type = MotionControl::kJoint;
        command.target_type = ControlType::kPosition;
        command.actuator_mode = ControlType::kPosition;
        // High follow ignores speed_ratio. A wire value of 0 is not safe there:
        // rt_control only rewrites 0 to full speed when acc_time > 0, and
        // raw-follow mode sends acc_time 0.
        const float speed_scale =
            (!follow && speed_ratio > 0)
                ? static_cast<float>(speed_ratio) / 100.0F
                : 1.0F;
        command.interpolation_speed_ratio = speed_scale;
        command.InterpolationConstVelTime = 0.0F;

        if (!follow) {
            command.arm_target_mode = PanelTargetMode::kSinglePoint;
            command.interpolation_type = InterpolationMethod::kQuintic;
            command.InterpolationAccTime =
                config["panel"]["general"]["acc_time"].as<float>();
        } else if (trajectory_mode == 0) {
            command.arm_target_mode = PanelTargetMode::kSinglePoint;
            command.interpolation_type = InterpolationMethod::kDirect;
            command.InterpolationAccTime = kMinimumProfileTime;
        } else if (trajectory_mode == 1) {
            command.arm_target_mode = PanelTargetMode::kSinglePoint;
            command.interpolation_type = InterpolationMethod::kQuintic;
            command.InterpolationAccTime = curve_fit_time(radio);
        } else {
            command.arm_target_mode = PanelTargetMode::kRawSinglePoint;
            command.interpolation_type = InterpolationMethod::kNone;
            command.NoneInterpolationSaturationRatio = filter_cutoff_frequency(radio);
            command.InterpolationAccTime = 0.0F;
        }

        command.reset_interpolation =
            previous_follow != follow || previous_trajectory_mode != trajectory_mode;
    }

    RetCode send_cached_joint_command(bool follow, uint8_t trajectory_mode,
                                      uint16_t radio,
                                      bool sticky_left_right = false) {
        if (!ensure_joint_target_cache()) return RetCode::RECEIVE_FAILED;
        if (follow && (trajectory_mode > 2 ||
                       (trajectory_mode == 1 && radio > 100) ||
                       (trajectory_mode == 2 && radio > 999)))
            return RetCode::CONTROLLER_ERROR;

        hold_unflushed_groups_in_cache(sticky_left_right);
        apply_joint_motion_profile(follow, trajectory_mode, radio);
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = joint_target_cache[arm][joint];

        synchronize_sequence_with_planner();
        const uint32_t sequence = command.sequence_id;
        const uint64_t generation_before_send = state_generation.load();
        if (!send_once()) return RetCode::SEND_FAILED;
        command.reset_interpolation = false;
        previous_follow = follow;
        previous_trajectory_mode = trajectory_mode;
        clear_arm_target_dirty();

        if (follow) return RetCode::SUCCESS;
        return wait_for_ack(sequence, generation_before_send)
                   ? RetCode::SUCCESS
                   : RetCode::TIMEOUT;
    }

    void clear_task_cmd() {
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int axis = 0; axis < 6; ++axis)
                command.TaskCmdDeg[arm][axis] = 0.0F;
    }

    void apply_task_line_profile(uint16_t speed_ratio, float acc_time) {
        command.motion_type = MotionControl::kTaskLine;
        command.task_orien_type = OrientControl::kEnd;
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        command.target_type = ControlType::kPosition;
        command.actuator_mode = ControlType::kPosition;
        command.reset_control_mem = true;
        command.interpolation_type = InterpolationMethod::kQuintic;
        command.interpolation_speed_ratio = clamp_speed_ratio(speed_ratio);
        command.InterpolationAccTime =
            acc_time > 0.0F ? acc_time
                            : config["panel"]["general"]["acc_time"].as<float>();
        command.InterpolationConstVelTime = 0.0F;
    }

    bool task_delta_near_zero(const std::vector<int>& arm_indices) const {
        for (int arm : arm_indices) {
            for (int axis = 0; axis < 6; ++axis) {
                const float eps = axis < 3 ? kTaskPosEpsM : kTaskOriEpsDeg;
                if (std::abs(command.TaskCmdDeg[arm][axis]) > eps) return false;
            }
        }
        return true;
    }

    static bool is_tool_motion_arm(int arm) { return arm == 0 || arm == 2; }

    void clear_tool_target_dirty() {
        for (int arm = 0; arm < MAX_ARM_SIZE; ++arm) tool_target_dirty[arm] = false;
    }

    void sync_tool_target_cache_from_state() {
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (!is_tool_motion_arm(arm)) continue;
            for (int i = 0; i < kToolPoseSize; ++i)
                tool_target_cache[arm][i] = snapshot.ToolPose[arm][i];
        }
        tool_target_cache_valid = true;
        clear_tool_target_dirty();
    }

    bool ensure_tool_target_cache() {
        if (tool_target_cache_valid) return true;
        if (!received_state.load()) return false;
        sync_tool_target_cache_from_state();
        return true;
    }

    void hold_unflushed_tool_targets_in_cache(bool sticky_left_right) {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (!is_tool_motion_arm(arm)) continue;
            if (sticky_left_right && is_tool_motion_arm(arm)) continue;
            if (tool_target_dirty[arm]) continue;
            for (int i = 0; i < kToolPoseSize; ++i)
                tool_target_cache[arm][i] = snapshot.ToolPose[arm][i];
        }
    }

    RetCode set_tool_line_target_locked(int arm, const std::vector<float>& tool_pose) {
        if (!is_tool_motion_arm(arm) ||
            tool_pose.size() != static_cast<size_t>(kToolPoseSize)) {
            return RetCode::CONTROLLER_ERROR;
        }
        if (!ensure_tool_target_cache()) return RetCode::RECEIVE_FAILED;
        for (int i = 0; i < kToolPoseSize; ++i)
            tool_target_cache[arm][i] = tool_pose[static_cast<size_t>(i)];
        tool_target_dirty[arm] = true;
        return RetCode::SUCCESS;
    }

    void hold_joints_at_feedback() {
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm)
            for (int joint = 0; joint < joint_sizes[arm]; ++joint)
                command.JointCmd[arm][joint] = snapshot.JointPos[arm][joint];
    }

    void fill_task_line_deltas_from_tool_cache(const PlannerState& snapshot) {
        clear_task_cmd();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (!is_tool_motion_arm(arm)) continue;
            float current_tool_pose[kToolPoseSize];
            float target_pose[kToolPoseSize];
            for (int i = 0; i < kToolPoseSize; ++i) {
                current_tool_pose[i] = snapshot.ToolPose[arm][i];
                target_pose[i] = tool_target_cache[arm][i];
            }
            fill_task_line_delta(command.TaskCmdDeg[arm], current_tool_pose,
                                 target_pose);
            for (int i = 0; i < kToolPoseSize; ++i)
                movel_target_tool_pose_[arm][i] = target_pose[i];
            movel_target_valid_[arm] = true;
        }
    }

    void fill_task_line_deltas_for_arms(const PlannerState& snapshot,
                                        const std::vector<int>& arm_indices,
                                        const std::vector<float>& target_tool_pose) {
        clear_task_cmd();
        size_t source = 0;
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            if (!is_tool_motion_arm(arm)) continue;

            float current_tool_pose[kToolPoseSize];
            for (int i = 0; i < kToolPoseSize; ++i)
                current_tool_pose[i] = snapshot.ToolPose[arm][i];

            float target_pose[kToolPoseSize];
            const bool user_targets_arm =
                std::find(arm_indices.begin(), arm_indices.end(), arm) !=
                arm_indices.end();
            if (user_targets_arm) {
                for (int i = 0; i < kToolPoseSize; ++i)
                    target_pose[i] = target_tool_pose[source++];
            } else {
                for (int i = 0; i < kToolPoseSize; ++i)
                    target_pose[i] = current_tool_pose[i];
            }

            fill_task_line_delta(command.TaskCmdDeg[arm], current_tool_pose,
                                 target_pose);
            for (int i = 0; i < kToolPoseSize; ++i)
                movel_target_tool_pose_[arm][i] = target_pose[i];
            movel_target_valid_[arm] = true;
        }
    }

    RetCode send_cached_tool_line_command(uint16_t speed_ratio, float acc_time,
                                          bool sticky_left_right) {
        if (!ensure_tool_target_cache()) return RetCode::RECEIVE_FAILED;

        hold_unflushed_tool_targets_in_cache(sticky_left_right);
        const PlannerState snapshot = state_snapshot();
        fill_task_line_deltas_from_tool_cache(snapshot);

        const std::vector<int> all_tool_arms = {0, 2};
        if (task_delta_near_zero(all_tool_arms)) {
            clear_tool_target_dirty();
            return RetCode::SUCCESS;
        }

        apply_task_line_profile(speed_ratio, acc_time);
        if (!tool_stream_profile_initialized) {
            command.reset_control_mem = true;
            tool_stream_profile_initialized = true;
        } else {
            command.reset_control_mem = false;
        }

        hold_joints_at_feedback();
        tool_stream_profile_initialized = false;
        return send_task_with_ack();
    }

    void request_tool_line_auto_flush_locked() {
        if (!tool_streaming_active.load()) return;
        tool_flush_pending = true;
    }

    void tool_auto_flush_loop() {
        while (tool_streaming_active.load()) {
            std::unique_lock<std::mutex> lock(command_mutex);
            tool_flush_cv.wait(lock, [&] {
                return tool_flush_pending || !tool_streaming_active.load();
            });
            if (!tool_streaming_active.load()) break;
            if (tool_coalesce_ms > 0) {
                tool_flush_cv.wait_for(
                    lock, std::chrono::milliseconds(tool_coalesce_ms));
            }
            tool_flush_pending = false;
            send_cached_tool_line_command(tool_stream_speed_ratio,
                                          tool_stream_acc_time, true);
        }
    }

    RetCode send_task_line_command(const std::vector<int>& arm_indices,
                                   const std::vector<float>& target_tool_pose,
                                   uint16_t speed_ratio,
                                   float acc_time) {
        if (!received_state.load()) return RetCode::RECEIVE_FAILED;
        if (target_tool_pose.size() !=
            arm_indices.size() * static_cast<size_t>(kToolPoseSize)) {
            return RetCode::CONTROLLER_ERROR;
        }

        const PlannerState snapshot = state_snapshot();
        apply_task_line_profile(speed_ratio, acc_time);
        command.reset_control_mem = true;
        fill_task_line_deltas_for_arms(snapshot, arm_indices, target_tool_pose);
        hold_joints_at_feedback();

        const std::vector<int> all_tool_arms = {0, 2};
        if (task_delta_near_zero(all_tool_arms)) return RetCode::SUCCESS;
        tool_stream_profile_initialized = false;
        return send_task_with_ack();
    }

    void switch_target_type(ControlType target_type, ControlType actuator_mode) {
        command.target_type = target_type;
        command.actuator_mode = actuator_mode;
        command.motion_type = MotionControl::kJoint;
        command.arm_target_mode = PanelTargetMode::kSinglePoint;
        if (!received_state.load()) return;
        const PlannerState snapshot = state_snapshot();
        for (int arm = 0; arm < command.ArmSize; ++arm) {
            for (int joint = 0; joint < joint_sizes[arm]; ++joint) {
                command.JointCmd[arm][joint] =
                    (target_type == ControlType::kPosition)
                        ? snapshot.JointPos[arm][joint]
                        : 0.0F;
            }
        }
    }

    RetCode start_teach_mode_locked(bool gravity_comp) {
        switch_target_type(ControlType::kTorque, ControlType::kTorque);
        if (gravity_comp) {
            command.motion_type = MotionControl::kControlAlgorithm;
            command.control_algorithm = ControlAlgorithm::kGravity;
        } else {
            command.motion_type = MotionControl::kJoint;
            command.control_algorithm = ControlAlgorithm::kNone;
        }
        command.need_setting_update = true;
        return send_task_with_ack();
    }

    RetCode stop_teach_mode_locked() {
        switch_target_type(ControlType::kPosition, ControlType::kPosition);
        command.control_algorithm = ControlAlgorithm::kNone;
        const RetCode result = send_task_with_ack();
        command.need_setting_update = true;
        return result;
    }
};

ControlApi::ControlApi(const std::string& config_prefix_path)
    : impl_(std::make_unique<Impl>(config_prefix_path)) {}

ControlApi::~ControlApi() {
    stop_dual_arm_tool_line_auto_flush();
    stop_dual_arm_streaming();
}

RetCode ControlApi::set_group(Group group, bool enable) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return RetCode::CONTROLLER_ERROR;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->command.enable_joint[arm][joint] = enable;
    return impl_->apply_settings();
}

std::tuple<RetCode, std::vector<State>> ControlApi::get_group_state(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};
    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<State> result;
    for (int arm : indices) {
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint) {
            if (snapshot.system_state == SystemState::kError)
                result.push_back(State::ERROR);
            else
                result.push_back(snapshot.enabled_joint[arm][joint] ? State::OPERATIONAL : State::SHUTDOWN);
        }
    }
    return {RetCode::SUCCESS, result};
}

RetCode ControlApi::move_line(Group group,
                              const std::vector<float>& target_tool_pose,
                              uint16_t speed_ratio, float acc_time) {
    if (impl_->streaming_active.load() || impl_->tool_streaming_active.load())
        return RetCode::CONTROLLER_ERROR;
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto arm_indices = impl_->tool_arm_indices(group);
    if (arm_indices.empty()) return RetCode::CONTROLLER_ERROR;
    if (target_tool_pose.size() != impl_->expected_tool_pose_count(group))
        return RetCode::CONTROLLER_ERROR;
    const RetCode sent = impl_->send_task_line_command(
        arm_indices, target_tool_pose, speed_ratio, acc_time);
    if (sent != RetCode::SUCCESS) return sent;
    return impl_->wait_until_reached(arm_indices, [&] {
        return impl_->tool_pose_reached(arm_indices, target_tool_pose);
    });
}

std::tuple<RetCode, std::vector<float>>
ControlApi::get_tool_pose(Group group) const {
    const auto arm_indices = impl_->tool_arm_indices(group);
    if (arm_indices.empty()) return {RetCode::CONTROLLER_ERROR, {}};
    if (!impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};

    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<float> result;
    result.reserve(impl_->expected_tool_pose_count(group));
    for (int arm : arm_indices) {
        for (int i = 0; i < kToolPoseSize; ++i)
            result.push_back(snapshot.ToolPose[arm][i]);
    }
    return {RetCode::SUCCESS, result};
}

RetCode ControlApi::move_joint(Group group, const std::vector<float>& joints,
                               bool follow, uint8_t trajectory_mode, uint16_t radio,
                               uint16_t speed_ratio) {
    if (impl_->tool_streaming_active.load()) return RetCode::CONTROLLER_ERROR;
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || joints.size() != impl_->expected_joint_count(group))
        return RetCode::CONTROLLER_ERROR;
    if (follow && (trajectory_mode > 2 ||
                   (trajectory_mode == 1 && radio > 100) ||
                   (trajectory_mode == 2 && radio > 999)))
        return RetCode::CONTROLLER_ERROR;
    if (!follow && speed_ratio > 100) return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    impl_->apply_joint_motion_profile(follow, trajectory_mode, radio, speed_ratio);
    impl_->hold_unselected_groups(indices);
    size_t source = 0;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->command.JointCmd[arm][joint] = joints[source++];

    impl_->synchronize_sequence_with_planner();
    const uint32_t sequence = impl_->command.sequence_id;
    const uint64_t generation_before_send = impl_->state_generation.load();
    if (!impl_->send_once()) return RetCode::SEND_FAILED;
    impl_->command.reset_interpolation = false;
    impl_->previous_follow = follow;
    impl_->previous_trajectory_mode = trajectory_mode;
    impl_->sync_joint_target_cache_from_command();

    // High-follow calls must remain non-blocking so the caller can maintain <=10 ms cadence.
    if (follow) return RetCode::SUCCESS;
    if (!impl_->wait_for_ack(sequence, generation_before_send))
        return RetCode::TIMEOUT;
    return impl_->wait_until_reached(indices, [&] {
        return impl_->joints_reached(indices, joints);
    });
}

RetCode ControlApi::set_joint_target(Group group,
                                     const std::vector<float>& joints) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    return impl_->set_joint_target_locked(group, joints);
}

RetCode ControlApi::flush_joint_command(bool follow, uint8_t trajectory_mode,
                                        uint16_t radio) {
    if (impl_->tool_streaming_active.load()) return RetCode::CONTROLLER_ERROR;
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    return impl_->send_cached_joint_command(follow, trajectory_mode, radio, false);
}

void ControlApi::reset_joint_target_cache() {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    impl_->joint_target_cache_valid = false;
    impl_->clear_arm_target_dirty();
}

RetCode ControlApi::set_left_joint_target(const std::vector<float>& joints) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    return impl_->set_joint_target_locked(Group::LEFT_ARM, joints);
}

RetCode ControlApi::set_right_joint_target(const std::vector<float>& joints) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    return impl_->set_joint_target_locked(Group::RIGHT_ARM, joints);
}

RetCode ControlApi::start_dual_arm_streaming(bool follow, uint8_t trajectory_mode,
                                             uint16_t radio, int hz) {
    if (hz <= 0 || hz > 1000) return RetCode::CONTROLLER_ERROR;
    if (follow && (trajectory_mode > 2 ||
                   (trajectory_mode == 1 && radio > 100) ||
                   (trajectory_mode == 2 && radio > 999)))
        return RetCode::CONTROLLER_ERROR;
    if (impl_->streaming_active.load() || impl_->tool_streaming_active.load())
        return RetCode::CONTROLLER_ERROR;

    {
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

        impl_->joint_target_cache_valid = false;
        impl_->clear_arm_target_dirty();
        if (!impl_->ensure_joint_target_cache()) return RetCode::RECEIVE_FAILED;

        impl_->streaming_follow = follow;
        impl_->streaming_trajectory_mode = trajectory_mode;
        impl_->streaming_radio = radio;
        impl_->streaming_hz = hz;
    }

    impl_->streaming_active.store(true);
    impl_->streaming_thread =
        std::thread(&Impl::streaming_loop, impl_.get());
    return RetCode::SUCCESS;
}

void ControlApi::stop_dual_arm_streaming() {
    impl_->stop_joint_streaming_loop();
}

bool ControlApi::is_dual_arm_streaming() const {
    return impl_->streaming_active.load();
}

RetCode ControlApi::set_left_tool_line_target(
    const std::vector<float>& tool_pose) {
    RetCode ret;
    {
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        ret = impl_->set_tool_line_target_locked(0, tool_pose);
        if (ret == RetCode::SUCCESS) impl_->request_tool_line_auto_flush_locked();
    }
    if (ret == RetCode::SUCCESS && impl_->tool_streaming_active.load()) {
        impl_->tool_flush_cv.notify_one();
    }
    return ret;
}

RetCode ControlApi::set_right_tool_line_target(
    const std::vector<float>& tool_pose) {
    RetCode ret;
    {
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        ret = impl_->set_tool_line_target_locked(2, tool_pose);
        if (ret == RetCode::SUCCESS) impl_->request_tool_line_auto_flush_locked();
    }
    if (ret == RetCode::SUCCESS && impl_->tool_streaming_active.load()) {
        impl_->tool_flush_cv.notify_one();
    }
    return ret;
}

RetCode ControlApi::flush_tool_line_command(uint16_t speed_ratio,
                                            float acc_time) {
    if (impl_->tool_streaming_active.load()) return RetCode::CONTROLLER_ERROR;
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    return impl_->send_cached_tool_line_command(speed_ratio, acc_time, false);
}

void ControlApi::reset_tool_line_target_cache() {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    impl_->tool_target_cache_valid = false;
    impl_->tool_stream_profile_initialized = false;
    impl_->clear_tool_target_dirty();
}

RetCode ControlApi::start_dual_arm_tool_line_auto_flush(uint16_t speed_ratio,
                                                        float acc_time,
                                                        int coalesce_ms) {
    if (coalesce_ms < 0 || coalesce_ms > 500) return RetCode::CONTROLLER_ERROR;
    if (impl_->streaming_active.load() || impl_->tool_streaming_active.load())
        return RetCode::CONTROLLER_ERROR;

    {
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

        impl_->tool_target_cache_valid = false;
        impl_->tool_stream_profile_initialized = false;
        impl_->tool_flush_pending = false;
        impl_->clear_tool_target_dirty();
        if (!impl_->ensure_tool_target_cache()) return RetCode::RECEIVE_FAILED;

        impl_->tool_stream_speed_ratio = speed_ratio;
        impl_->tool_stream_acc_time = acc_time;
        impl_->tool_coalesce_ms = coalesce_ms;
    }

    impl_->tool_streaming_active.store(true);
    impl_->tool_streaming_thread =
        std::thread(&Impl::tool_auto_flush_loop, impl_.get());
    return RetCode::SUCCESS;
}

void ControlApi::stop_dual_arm_tool_line_auto_flush() {
    {
        std::lock_guard<std::mutex> lock(impl_->command_mutex);
        impl_->tool_flush_pending = true;
    }
    impl_->tool_flush_cv.notify_all();
    impl_->stop_tool_streaming_loop();
}

bool ControlApi::is_dual_arm_tool_line_auto_flush() const {
    return impl_->tool_streaming_active.load();
}

std::tuple<RetCode, std::vector<float>> ControlApi::get_joint(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};
    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<float> result;
    result.reserve(impl_->expected_joint_count(group));
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(snapshot.JointPos[arm][joint]);
    return {RetCode::SUCCESS, result};
}

std::tuple<RetCode, std::vector<float>>
ControlApi::get_joint_velocity(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || !impl_->received_state.load())
        return {RetCode::RECEIVE_FAILED, {}};

    const PlannerState snapshot = impl_->state_snapshot();
    std::vector<float> result;
    result.reserve(impl_->expected_joint_count(group));
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(snapshot.JointVel[arm][joint]);
    return {RetCode::SUCCESS, result};
}

std::vector<float> ControlApi::set_joint_max_speed(Group group, const std::vector<float>& limits) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || limits.size() != impl_->expected_joint_count(group)) return {};
    size_t source = 0;
    for (int arm : indices) {
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint) {
            const float value = std::abs(limits[source++]);
            impl_->command.arm_soft_limit_velocity[arm][joint][0] = -value;
            impl_->command.arm_soft_limit_velocity[arm][joint][1] = value;
        }
    }
    return impl_->apply_settings() == RetCode::SUCCESS ? limits : std::vector<float>{};
}

std::vector<float> ControlApi::set_joint_max_acc(Group group, const std::vector<float>& limits) {
    const auto indices = impl_->group_indices(group);
    if (indices.empty() || limits.size() != impl_->expected_joint_count(group)) return {};
    size_t source = 0;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            impl_->max_acceleration[arm][joint] = std::abs(limits[source++]);
    return limits;
}

std::vector<float> ControlApi::get_joint_max_limit(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return {};
    std::vector<float> result;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(impl_->command.arm_soft_limit_position[arm][joint][1]);
    return result;
}

std::vector<float> ControlApi::get_joint_min_limit(Group group) const {
    const auto indices = impl_->group_indices(group);
    if (indices.empty()) return {};
    std::vector<float> result;
    for (int arm : indices)
        for (int joint = 0; joint < impl_->joint_sizes[arm]; ++joint)
            result.push_back(impl_->command.arm_soft_limit_position[arm][joint][0]);
    return result;
}

RetCode ControlApi::set_gripper_config(Group group, uint16_t speed,
                                       uint16_t force, bool block,
                                       int timeout) {
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0 || speed < CUARM_GRIPPER_SPEED_MIN ||
        speed > CUARM_GRIPPER_SPEED_MAX ||
        force < CUARM_GRIPPER_FORCE_MIN ||
        force > CUARM_GRIPPER_FORCE_MAX ||
        (block && timeout <= 0))
        return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    impl_->command.GripperSpeed[gripper] = speed;
    impl_->command.GripperForce[gripper] = force;
    const RetCode initialization = impl_->ensure_initialized();
    if (initialization != RetCode::SUCCESS) return initialization;
    if (block)
        return impl_->send_with_ack(timeout * 1000);

    impl_->synchronize_sequence_with_planner();
    return impl_->send_once() ? RetCode::SUCCESS : RetCode::SEND_FAILED;
}

RetCode ControlApi::set_gripper_position(Group group, uint16_t position,
                                         bool block, int timeout) {
    std::unique_lock<std::mutex> lock(impl_->command_mutex);
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0 || position < CUARM_GRIPPER_POSITION_MIN ||
        position > CUARM_GRIPPER_POSITION_MAX ||
        (block && timeout <= 0))
        return RetCode::CONTROLLER_ERROR;
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;

    const RetCode initialization = impl_->ensure_initialized();
    if (initialization != RetCode::SUCCESS) return initialization;

    impl_->command.GripperJointCmd[gripper][0] =
        static_cast<float>(position) /
        static_cast<float>(CUARM_GRIPPER_POSITION_MAX);
    const uint32_t gripper_command =
        ++impl_->command.GripperCommandId[gripper];

    impl_->synchronize_sequence_with_planner();
    const uint32_t sequence = impl_->command.sequence_id;
    const uint64_t generation_before_send = impl_->state_generation.load();
    if (!impl_->send_once()) return RetCode::SEND_FAILED;
    if (!block) return RetCode::SUCCESS;

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(timeout);
    if (!impl_->wait_for_ack(
            sequence, generation_before_send,
            std::min(kAckTimeoutMs, std::max(1, timeout * 1000))))
        return RetCode::TIMEOUT;

    while (std::chrono::steady_clock::now() < deadline) {
        const PlannerState snapshot = impl_->state_snapshot();
        if (snapshot.GripperCommandId[gripper] >= gripper_command) {
            if (snapshot.GripperStatus[gripper] == 2)
                return RetCode::CONTROLLER_ERROR;
            if (snapshot.GripperStatus[gripper] == 1)
                return RetCode::SUCCESS;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return RetCode::TIMEOUT;
}

std::tuple<RetCode, std::unordered_map<std::string, uint16_t>>
ControlApi::get_gripper_state(Group group) const {
    const int gripper = impl_->gripper_index(group);
    if (gripper < 0) return {RetCode::CONTROLLER_ERROR, {}};
    if (!impl_->received_state.load()) return {RetCode::RECEIVE_FAILED, {}};

    const PlannerState snapshot = impl_->state_snapshot();
    if (gripper >= snapshot.GripperSize)
        return {RetCode::RECEIVE_FAILED, {}};
    if (snapshot.GripperStatus[gripper] == 2)
        return {RetCode::CONTROLLER_ERROR, {}};

    const float normalized = std::clamp(
        snapshot.GripperJointPos[gripper][0], 0.0F, 1.0F);
    const uint16_t position = static_cast<uint16_t>(std::lround(
        normalized * static_cast<float>(CUARM_GRIPPER_POSITION_MAX)));
    return {RetCode::SUCCESS,
            {{"position", position},
             {"force", snapshot.GripperForce[gripper]},
             {"speed", snapshot.GripperSpeed[gripper]}}};
}

RetCode ControlApi::start_teach_mode(bool gravity_comp) {
    stop_dual_arm_tool_line_auto_flush();
    stop_dual_arm_streaming();
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;
    return impl_->start_teach_mode_locked(gravity_comp);
}

RetCode ControlApi::stop_teach_mode() {
    stop_dual_arm_tool_line_auto_flush();
    stop_dual_arm_streaming();
    std::lock_guard<std::mutex> lock(impl_->command_mutex);
    if (!impl_->received_state.load()) return RetCode::RECEIVE_FAILED;
    RetCode result = impl_->stop_teach_mode_locked();
    if (result != RetCode::SUCCESS) return result;
    impl_->hold_all_groups_at_current_position();
    impl_->sync_joint_target_cache_from_command();
    return RetCode::SUCCESS;
}

}  // namespace dual_arm_v2_2_sdk
