#ifndef DUAL_ARM_V2_2_CONTROL_API_H
#define DUAL_ARM_V2_2_CONTROL_API_H

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace dual_arm_v2_2_sdk {

enum class Group : uint8_t {
    ALL = 1,
    LEFT_ARM = 2,
    RIGHT_ARM = 3,
    HEAD = 4,
    ELEVATOR = 5,
};

enum class RetCode : int {
    SUCCESS = 0,
    CONTROLLER_ERROR = 1,
    SEND_FAILED = -1,
    RECEIVE_FAILED = -2,
    PARSE_FAILED = -3,
    TIMEOUT = -4,
};

enum class State : uint8_t {
    SHUTDOWN = 0,
    OPERATIONAL = 1,
    ERROR = 2,
};

class ControlApi {
public:
    explicit ControlApi(const std::string& config_prefix_path);
    ~ControlApi();

    ControlApi(const ControlApi&) = delete;
    ControlApi& operator=(const ControlApi&) = delete;

    RetCode set_group(Group group, bool enable);
    std::tuple<RetCode, std::vector<State>> get_group_state(Group group) const;

    RetCode move_joint(Group group, const std::vector<float>& joints,
                       bool follow = false, uint8_t trajectory_mode = 0,
                       uint16_t radio = 0);

    // Cartesian line move (MoveL), matching dual_arm_sdk moveToolLine / kTaskLine.
    // target_tool_pose layout per arm: x, y, z (m), qw, qx, qy, qz — same frame as
    // get_tool_pose() / PlannerState ToolPose. LEFT_ARM or RIGHT_ARM -> 7 floats;
    // ALL -> 14 floats (left arm then right arm; head is held with zero task delta).
    RetCode move_line(Group group, const std::vector<float>& target_tool_pose,
                      uint16_t speed_ratio = 50, float acc_time = 0.0f);

    // Read current tool pose feedback (7 floats per arm: x,y,z,qw,qx,qy,qz).
    std::tuple<RetCode, std::vector<float>> get_tool_pose(Group group) const;

    // Coalesced dual-arm streaming: update cached targets without sending.
    // Call set_joint_target() for each group, then flush_joint_command() once
    // per control cycle. Groups not updated since the last flush are held at
    // their current feedback pose when the packet is sent.
    RetCode set_joint_target(Group group, const std::vector<float>& joints);
    RetCode flush_joint_command(bool follow = false, uint8_t trajectory_mode = 0,
                                uint16_t radio = 0);
    void reset_joint_target_cache();

    // Independent dual-arm streaming (e.g. async left/right task loops).
    // start_dual_arm_streaming() runs an internal 100 Hz sender; each cycle
    // emits one packet with the latest left/right cache. Unupdated arms keep
    // their last target (not feedback-hold). Head uses feedback when not dirty.
    RetCode set_left_joint_target(const std::vector<float>& joints);
    RetCode set_right_joint_target(const std::vector<float>& joints);
    RetCode start_dual_arm_streaming(bool follow = true,
                                     uint8_t trajectory_mode = 2,
                                     uint16_t radio = 999, int hz = 100);
    void stop_dual_arm_streaming();
    bool is_dual_arm_streaming() const;

    // Independent dual-arm MoveL (ToolPose, kTaskLine). Layout per arm:
    // x,y,z (m), qw,qx,qy,qz — same as get_tool_pose() / move_line.
    //
    // Low-follow coalesced: set_left/right_tool_line_target(), then
    // start_dual_arm_tool_line_auto_flush(). One UDP packet merges L+R TaskCmd;
    // send_task_with_ack (blocking). Unupdated arm keeps last cache (sticky).
    // Or set targets and call flush_tool_line_command() once (no background thread).
    RetCode set_left_tool_line_target(const std::vector<float>& tool_pose);
    RetCode set_right_tool_line_target(const std::vector<float>& tool_pose);
    RetCode flush_tool_line_command(uint16_t speed_ratio = 50, float acc_time = 0.0f);
    void reset_tool_line_target_cache();
    RetCode start_dual_arm_tool_line_auto_flush(uint16_t speed_ratio = 50,
                                                float acc_time = 0.0f,
                                                int coalesce_ms = 10);
    void stop_dual_arm_tool_line_auto_flush();
    bool is_dual_arm_tool_line_auto_flush() const;

    std::tuple<RetCode, std::vector<float>> get_joint(Group group) const;
    std::tuple<RetCode, std::vector<float>> get_joint_velocity(Group group) const;

    std::vector<float> set_joint_max_speed(Group group, const std::vector<float>& joint_limit);
    // Not implemented yet: returns SDK-local cache only; limits are not sent to
    // rt_control. Configure acceleration via config.yaml
    // robot.arm[].safety.joint_soft_limit.acceleration instead.
    std::vector<float> set_joint_max_acc(Group group, const std::vector<float>& joint_limit);
    std::vector<float> get_joint_max_limit(Group group) const;
    std::vector<float> get_joint_min_limit(Group group) const;

    // Gripper values are normalized integers: position 0=open, 1000=closed;
    // speed 1..1000 maps to 0.36..360 deg/s; force 1..1000 maps to
    // 0.001..1.0 Nm. LEFT_ARM maps to gripperL, RIGHT_ARM to gripperR.
    RetCode set_gripper_config(Group group, uint16_t speed, uint16_t force,
                               bool block, int timeout);
    RetCode set_gripper_position(Group group, uint16_t position,
                                 bool block, int timeout);
    std::tuple<RetCode, std::unordered_map<std::string, uint16_t>>
    get_gripper_state(Group group) const;

    // Gravity-compensated teach mode (torque control + kGravity on rt_control).
    // Requires force_control.gravity configured in robot YAML. Stops joint/tool
    // streaming if active. Pair with stop_teach_mode() before move_joint replay.
    RetCode start_teach_mode(bool gravity_comp = true);
    RetCode stop_teach_mode();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace dual_arm_v2_2_sdk

#endif
