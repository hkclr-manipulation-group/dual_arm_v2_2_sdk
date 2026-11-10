// Python binding for the Dual Arm V2.2 ControlApi.
//
// The exposed surface matches the C++ public API except set_joint_max_acc,
// which only fills an SDK-local cache and is intentionally not bound.
// Keep the Python section of doc/control_api.md in sync with this file.

#ifndef DUAL_ARM_PY_VERSION
#define DUAL_ARM_PY_VERSION 1.0.0
#endif
#define DUAL_ARM_PY_STRINGIFY_IMPL(x) #x
#define DUAL_ARM_PY_STRINGIFY(x) DUAL_ARM_PY_STRINGIFY_IMPL(x)

#include "control_api.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace dual_arm_v2_2_sdk;

PYBIND11_MODULE(dual_arm_v2_2_sdk, m) {
    m.doc() = "Python binding for the Dual Arm V2.2 ControlApi";
    m.attr("__version__") = DUAL_ARM_PY_STRINGIFY(DUAL_ARM_PY_VERSION);

    py::enum_<Group>(m, "Group")
        .value("ALL", Group::ALL)
        .value("LEFT_ARM", Group::LEFT_ARM)
        .value("RIGHT_ARM", Group::RIGHT_ARM)
        .value("HEAD", Group::HEAD)
        .value("ELEVATOR", Group::ELEVATOR)
        .export_values();

    py::enum_<RetCode>(m, "RetCode")
        .value("SUCCESS", RetCode::SUCCESS)
        .value("CONTROLLER_ERROR", RetCode::CONTROLLER_ERROR)
        .value("SEND_FAILED", RetCode::SEND_FAILED)
        .value("RECEIVE_FAILED", RetCode::RECEIVE_FAILED)
        .value("PARSE_FAILED", RetCode::PARSE_FAILED)
        .value("TIMEOUT", RetCode::TIMEOUT)
        .export_values();

    py::enum_<State>(m, "State")
        .value("SHUTDOWN", State::SHUTDOWN)
        .value("OPERATIONAL", State::OPERATIONAL)
        .value("ERROR", State::ERROR)
        .export_values();

    py::class_<ControlApi>(m, "ControlApi")
        .def(py::init<const std::string&>(), py::arg("config_prefix_path"),
             "config_prefix_path points at the dual_v2_2 configuration directory "
             "(the folder containing config.yaml).")

        // --- B. Enable / state ---
        .def("set_group", &ControlApi::set_group, py::arg("group"), py::arg("enable"),
             py::call_guard<py::gil_scoped_release>())
        .def("get_group_state", &ControlApi::get_group_state, py::arg("group"),
             "Returns (RetCode, [State, ...]) per joint of the group.")

        // --- C. Joint space: direct move ---
        .def("move_joint", &ControlApi::move_joint, py::arg("group"),
             py::arg("joints"), py::arg("follow") = false,
             py::arg("trajectory_mode") = 0, py::arg("radio") = 0,
             py::arg("speed_ratio") = 0,
             py::call_guard<py::gil_scoped_release>(),
             "Moves one group to an absolute joint target (rad). follow=False "
             "waits for the controller ack, then blocks until each commanded "
             "joint is within 0.01 deg for 3 samples (20 s timeout). speed_ratio "
             "0 keeps the full joint-velocity soft limit; 1..100 scales it, and "
             "values above 100 are rejected. follow=True returns after the packet "
             "is sent and must be re-sent at <=10 ms; speed_ratio is ignored. "
             "Arms outside the group are held at their feedback pose.")

        // --- E. Cartesian line: direct MoveL (blocks until the tool pose is reached) ---
        // target_tool_pose: 7 floats per arm (x,y,z m + qw,qx,qy,qz). LEFT_ARM or
        // RIGHT_ARM -> 7 values; ALL -> 14 (left then right).
        .def("move_line", &ControlApi::move_line, py::arg("group"),
             py::arg("target_tool_pose"), py::arg("speed_ratio") = 50,
             py::arg("acc_time") = 0.0f, py::call_guard<py::gil_scoped_release>(),
             "Blocking Cartesian straight line (kTaskLine) to an absolute tool "
             "pose. After the controller ack, waits until each commanded tool is "
             "within 0.5 mm and 0.01 deg for 3 samples, or TIMEOUT at 20 s. "
             "Rejected while joint or tool-line auto-flush streaming is active.")

        // --- D. Joint streaming: cache + manual flush + fixed-rate sender ---
        .def("set_joint_target", &ControlApi::set_joint_target, py::arg("group"),
             py::arg("joints"), py::call_guard<py::gil_scoped_release>())
        .def("set_left_joint_target", &ControlApi::set_left_joint_target,
             py::arg("joints"), py::call_guard<py::gil_scoped_release>())
        .def("set_right_joint_target", &ControlApi::set_right_joint_target,
             py::arg("joints"), py::call_guard<py::gil_scoped_release>())
        .def("flush_joint_command", &ControlApi::flush_joint_command,
             py::arg("follow") = false, py::arg("trajectory_mode") = 0,
             py::arg("radio") = 0, py::call_guard<py::gil_scoped_release>(),
             "Sends the cached joint targets once. Blocking until ACK.")
        .def("reset_joint_target_cache", &ControlApi::reset_joint_target_cache)
        .def("start_dual_arm_streaming", &ControlApi::start_dual_arm_streaming,
             py::arg("follow") = true, py::arg("trajectory_mode") = 2,
             py::arg("radio") = 999, py::arg("hz") = 100,
             py::call_guard<py::gil_scoped_release>(),
             "Starts the internal fixed-rate (hz) sender for left/right joint cache.")
        .def("stop_dual_arm_streaming", &ControlApi::stop_dual_arm_streaming,
             py::call_guard<py::gil_scoped_release>())
        .def("is_dual_arm_streaming", &ControlApi::is_dual_arm_streaming)

        // --- F. Tool-line (kTaskLine) streaming: cache + manual flush + coalesced sender ---
        // tool_pose: 7 floats per arm, x, y, z (m), qw, qx, qy, qz.
        .def("set_left_tool_line_target", &ControlApi::set_left_tool_line_target,
             py::arg("tool_pose"), py::call_guard<py::gil_scoped_release>(),
             "Caches the left-arm target pose (7 floats). Sends nothing by itself.")
        .def("set_right_tool_line_target", &ControlApi::set_right_tool_line_target,
             py::arg("tool_pose"), py::call_guard<py::gil_scoped_release>(),
             "Caches the right-arm target pose (7 floats). Sends nothing by itself.")
        .def("flush_tool_line_command", &ControlApi::flush_tool_line_command,
             py::arg("speed_ratio") = 50, py::arg("acc_time") = 0.0f,
             py::call_guard<py::gil_scoped_release>(),
             "Sends both cached tool targets once (non-dirty arms are held at "
             "feedback). Rejected while the auto-flush thread is running.")
        .def("reset_tool_line_target_cache", &ControlApi::reset_tool_line_target_cache)
        .def("start_dual_arm_tool_line_auto_flush",
             &ControlApi::start_dual_arm_tool_line_auto_flush,
             py::arg("speed_ratio") = 50, py::arg("acc_time") = 0.0f,
             py::arg("coalesce_ms") = 10, py::call_guard<py::gil_scoped_release>(),
             "Starts an event-driven sender that merges left/right updates inside a "
             "coalesce_ms window and sends one packet per window.")
        .def("stop_dual_arm_tool_line_auto_flush",
             &ControlApi::stop_dual_arm_tool_line_auto_flush,
             py::call_guard<py::gil_scoped_release>())
        .def("is_dual_arm_tool_line_auto_flush", &ControlApi::is_dual_arm_tool_line_auto_flush)

        // --- G. Joint limits / speed / acc (re-exported in 0.1.3) ---
        // A waypoint tool reads the configured soft limits before it moves and
        // pushes a joint velocity ceiling to rt_control first.
        // set_joint_max_acc stays unbound on purpose: it only fills an
        // SDK-local cache and never reaches rt_control -- configure
        // acceleration via config.yaml robot.arm[].safety.joint_soft_limit.acceleration.
        .def("get_joint_min_limit", &ControlApi::get_joint_min_limit,
             py::arg("group"),
             "Returns the configured lower soft limits (rad) of the group, [] for "
             "an unknown group.")
        .def("get_joint_max_limit", &ControlApi::get_joint_max_limit,
             py::arg("group"),
             "Returns the configured upper soft limits (rad) of the group, [] for "
             "an unknown group.")
        .def("set_joint_max_speed", &ControlApi::set_joint_max_speed,
             py::arg("group"), py::arg("joint_limit"),
             py::call_guard<py::gil_scoped_release>(),
             "Sets the joint velocity ceiling (rad/s, one entry per joint) of the "
             "group and pushes it to rt_control in a settings packet. Returns the "
             "applied limits, or [] when the controller rejects it.")

        // --- Feedback queries (needed to close the streaming loops) ---
        .def("get_joint", &ControlApi::get_joint, py::arg("group"),
             "Returns (RetCode, [joint positions in rad]).")
        .def("get_joint_velocity", &ControlApi::get_joint_velocity, py::arg("group"),
             "Returns (RetCode, [joint velocities in rad/s]).")
        .def("get_tool_pose", &ControlApi::get_tool_pose, py::arg("group"),
             "Returns (RetCode, [x, y, z, qw, qx, qy, qz] per arm).")

        // --- H. Gripper ---
        .def("set_gripper_config", &ControlApi::set_gripper_config, py::arg("group"),
             py::arg("speed"), py::arg("force"), py::arg("block"), py::arg("timeout"),
             py::call_guard<py::gil_scoped_release>())
        .def("set_gripper_position", &ControlApi::set_gripper_position, py::arg("group"),
             py::arg("position"), py::arg("block"), py::arg("timeout"),
             py::call_guard<py::gil_scoped_release>())
        .def("get_gripper_state", &ControlApi::get_gripper_state, py::arg("group"),
             "Returns (RetCode, {name: value}) with position/force/speed.")

        // --- I. Teach mode (gravity compensation on both arms) ---
        // Hands-on teaching: both arms switch to torque control so the operator
        // can push them by hand.  Cancels any running joint/tool streaming first.
        .def("start_teach_mode", &ControlApi::start_teach_mode,
             py::arg("gravity_comp") = true,
             py::call_guard<py::gil_scoped_release>(),
             "Enters teach mode on both arms (torque control). gravity_comp=True "
             "compensates gravity (arms float, drivable by hand); False gives "
             "zero-torque free fall. Stops joint streaming and tool-line "
             "auto-flush first. Blocking until the controller acks; returns "
             "RECEIVE_FAILED when no planner state has been received yet. Needs "
             "robot.arm[].force_control.gravity in the config.")
        .def("stop_teach_mode", &ControlApi::stop_teach_mode,
             py::call_guard<py::gil_scoped_release>(),
             "Leaves teach mode: back to position control, both arms are held at "
             "their current position and the joint cache is re-seeded from "
             "feedback, so a following move_joint starts from the taught pose. "
             "Blocking until the controller acks.");
}
