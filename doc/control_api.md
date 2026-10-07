# Dual Arm V2.2 控制 API 参考

**版本**：基于 `control_api_draft_20260902v1`，合并 2026 年新接口  
**命名空间**：`dual_arm_v2_2_sdk`  
**头文件**：

- C++：[`cpp/include/control_api.h`](../cpp/include/control_api.h)
- C：[`c/include/dual_arm_control_api.h`](../c/include/dual_arm_control_api.h)

**配置路径**：构造时传入 `cuarm_configuration/dual_v2_2` 目录（含 `config.yaml`）。

---

## 1. 概述

SDK 通过 UDP 与 `rt_control` 通信：

- 从 `panel.ip:port` 接收关节/系统状态
- 向 `rt_control.ip:port` 发送运动与夹爪指令

**单位约定**：

- 关节角度、速度、限位：**弧度（rad）**
- 夹爪 position / speed / force：**归一化整数**（见第 10 节）

**关节组与向量顺序**：

| `Group` | 对应机械臂索引 | 关节数（典型 dual_v2_2） | 向量顺序 |
|---------|----------------|--------------------------|----------|
| `LEFT_ARM` | arm 0 | 7 | 左臂 7 关节 |
| `HEAD` | arm 1 | 2 | 头部 2 关节 |
| `RIGHT_ARM` | arm 2 | 7 | 右臂 7 关节 |
| `ALL` | 0 + 1 + 2 | 16 | **左(7) + 头(2) + 右(7)** |
| `ELEVATOR` | — | — | **暂不支持** |

---

## 2. 公共类型

### 2.1 `Group`

```cpp
enum class Group : uint8_t {
    ALL = 1,
    LEFT_ARM = 2,
    RIGHT_ARM = 3,
    HEAD = 4,
    ELEVATOR = 5,
};
```

C 对应：`dual_arm_group_t`（`DUAL_ARM_GROUP_*`）。

### 2.2 `RetCode`

| 值 | 名称 | 含义 |
|----|------|------|
| `0` | `SUCCESS` | 成功 |
| `1` | `CONTROLLER_ERROR` | 控制器拒绝；参数错误或机械臂状态异常 |
| `-1` | `SEND_FAILED` | UDP 发送失败 |
| `-2` | `RECEIVE_FAILED` | 未收到有效状态或接收失败 |
| `-3` | `PARSE_FAILED` | 返回值解析失败 |
| `-4` | `TIMEOUT` | 等待控制器确认超时（低跟随 / 阻塞夹爪） |

C 对应：`dual_arm_ret_code_t`（`DUAL_ARM_RET_*`）。  
辅助函数：`dual_arm_ret_code_name()`、`dual_arm_control_api_last_error()`。

### 2.3 `State`（单关节使能状态）

| 值 | 名称 | 含义 |
|----|------|------|
| `0` | `SHUTDOWN` | 未使能 |
| `1` | `OPERATIONAL` | 已使能 |
| `2` | `ERROR` | 系统错误 |

`get_group_state` 按组内关节顺序返回与关节数相同长度的 `State` 向量。

---

## 3. 对象生命周期

### C++

```cpp
explicit ControlApi(const std::string& config_prefix_path);
~ControlApi();  // 自动 stop_dual_arm_streaming()
```

### C

```c
dual_arm_control_api_t* dual_arm_control_api_create(const char* config_prefix_path);
void dual_arm_control_api_destroy(dual_arm_control_api_t* api);
```

**注意**：首次调用运动/读状态前，应等待 `get_joint()` 返回 `SUCCESS`（表示已收到 `rt_control` 状态）。

---

## 4. 组使能

### `set_group`

```cpp
RetCode set_group(Group group, bool enable);
```

| 参数 | 说明 |
|------|------|
| `group` | 目标组（`ALL` / `LEFT_ARM` / `RIGHT_ARM` / `HEAD`） |
| `enable` | `true` 使能远程控制；`false` 关闭 |

使能时会下发 settings 包，并以**当前反馈位姿**作为首帧目标，避免零位指令。

C：`dual_arm_set_group(api, group, enable)`（`enable` 为 `int` 0/1）。

---

## 5. 关节运动：`move_joint`（单次发包）

```cpp
RetCode move_joint(Group group,
                   const std::vector<float>& joints,
                   bool follow = false,
                   uint8_t trajectory_mode = 0,
                   uint16_t radio = 0);
```

| 参数 | 说明 |
|------|------|
| `group` | 目标组 |
| `joints` | 目标关节角（rad），长度须等于该组关节总数 |
| `follow` | `true` 高跟随；`false` 低跟随（quintic 单点） |
| `trajectory_mode` | 仅 `follow=true` 时有效，见下表 |
| `radio` | 曲线拟合 / 滤波平滑系数 |

**行为**：

- 未选中的 arm 组保持**当前反馈位姿**（`hold_unselected_groups`）
- `follow=true`：**非阻塞**，立即返回，调用方须维持 **≤10 ms** 发包周期
- `follow=false`：阻塞等待 ACK（默认超时约 200 ms）

### 高跟随模式映射

| `follow` | `trajectory_mode` | rt_control 模式 | 插值 | `radio` 范围与作用 |
|----------|-------------------|-----------------|------|-------------------|
| `false` | — | `kSinglePoint` | `kQuintic` | 低跟随，acc_time 来自 `config.yaml` |
| `true` | `0` | `kSinglePoint` | `kDirect` | 完全透传 |
| `true` | `1` | `kSinglePoint` | `kQuintic` | **0..100**，越大曲线拟合越平滑（acc_time ≈ 10 ms..1.01 s） |
| `true` | `2` | `kRawSinglePoint` | `kNone` + 低通 | **0..999**，截止频率对数映射 20 Hz..1 Hz |

滤波截止频率：

```text
cutoff_hz = 20 * (1/20)^(radio/999)
```

C：`dual_arm_move_joint(api, group, joints, joint_count, follow, trajectory_mode, radio)`。

---

## 5.1 笛卡尔直线：`move_line`（MoveL / `kTaskLine`）

```cpp
RetCode move_line(Group group,
                  const std::vector<float>& target_tool_pose,
                  uint16_t speed_ratio = 50,
                  float acc_time = 0.0f);

std::tuple<RetCode, std::vector<float>> get_tool_pose(Group group) const;
```

| 参数 | 说明 |
|------|------|
| `group` | `LEFT_ARM` / `RIGHT_ARM` / `ALL`（`HEAD` 不支持） |
| `target_tool_pose` | 每臂 7 个数：`x,y,z`（m）+ `qw,qx,qy,qz`；单臂 7 个，`ALL` 为左 7 + 右 14 |
| `speed_ratio` | 0..100，映射到 `interpolation_speed_ratio` |
| `acc_time` | 加速时间（s）；0 表示用 `config.yaml` 默认值 |

**行为**（与 `dual_arm_sdk` 的 `moveToolLine` 一致）：

- 从 `PlannerState::ToolPose` 读当前位姿，计算相对增量写入 `TaskCmdDeg`
- 未参与运动的 arm：`TaskCmdDeg = 0`（保持不动）
- `motion_type = kTaskLine`，阻塞等待 ACK
- **dual-arm streaming 激活时**返回 `CONTROLLER_ERROR`

C：

- `dual_arm_move_line(api, group, pose, pose_count, speed_ratio, acc_time)`
- `dual_arm_get_tool_pose(api, group, out_pose, capacity, &count)`

示例：`examples/cpp/test_move_line.cpp`、`examples/c/test_move_line.c`。

---

## 6. 关节运动：合并发包（Coalesced Flush）

适用于**同一控制周期内**更新左/右（或头）目标，**只发一个 UDP 包**。

### 6.1 `set_joint_target`

```cpp
RetCode set_joint_target(Group group, const std::vector<float>& joints);
```

仅更新 SDK 内部 **joint target cache**，**不发送** UDP。

### 6.2 `flush_joint_command`

```cpp
RetCode flush_joint_command(bool follow = false,
                            uint8_t trajectory_mode = 0,
                            uint16_t radio = 0);
```

将 cache 写入 `JointCmd` 并发送一包。参数语义同 `move_joint` 的 `follow/trajectory_mode/radio`。

**未在本周期 `set_joint_target` 的 arm**（`arm_target_dirty[arm]==false`）：flush 时用**当前反馈位姿**填充（feedback hold）。

### 6.3 `reset_joint_target_cache`

```cpp
void reset_joint_target_cache();
```

清空 cache 有效标志与 dirty 标记；下次 flush 前会从反馈重新初始化 cache。

### 典型用法（100 Hz 双臂）

```cpp
api.set_joint_target(Group::LEFT_ARM,  left_joints);
api.set_joint_target(Group::RIGHT_ARM, right_joints);
api.flush_joint_command(true, 2, 999);  // 每 10 ms 一次
```

示例：`examples/cpp/test_dual_arm_100hz_follow.cpp` 模式 **`both-flush`**。

C：`dual_arm_set_joint_target`、`dual_arm_flush_joint_command`、`dual_arm_reset_joint_target_cache`。

---

## 7. 关节运动：独立双臂 Streaming

适用于**左/右在不同线程或异步循环**中各自更新目标；SDK 内部线程按固定频率发包，**无需用户每周期 flush**。

### 7.1 `set_left_joint_target` / `set_right_joint_target`

```cpp
RetCode set_left_joint_target(const std::vector<float>& joints);   // 7 rad
RetCode set_right_joint_target(const std::vector<float>& joints);  // 7 rad
```

仅更新对应臂 cache，不发送。

### 7.2 `start_dual_arm_streaming`

```cpp
RetCode start_dual_arm_streaming(bool follow = true,
                                 uint8_t trajectory_mode = 2,
                                 uint16_t radio = 999,
                                 int hz = 100);
```

| 参数 | 默认 | 说明 |
|------|------|------|
| `follow` | `true` | 高跟随 |
| `trajectory_mode` | `2` | 推荐滤波模式 |
| `radio` | `999` | 滤波平滑（最柔） |
| `hz` | `100` | 内部发送频率（1..1000） |

启动前会从当前反馈初始化 cache。若已在 streaming，返回 `CONTROLLER_ERROR`。

### 7.3 `stop_dual_arm_streaming` / `is_dual_arm_streaming`

```cpp
void stop_dual_arm_streaming();
bool is_dual_arm_streaming() const;
```

析构时自动 `stop`。

### 与 Coalesced Flush 的行为差异

| 场景 | Coalesced Flush | Independent Streaming |
|------|-----------------|------------------------|
| 本周期未更新的左/右臂 | **反馈 hold** | **保持上次目标**（sticky） |
| 头部未更新 | 反馈 hold | 反馈 hold |
| 发包方式 | 用户调用 `flush_joint_command` | SDK 内部线程 `@hz` |
| 典型场景 | 单线程 100 Hz 同步双臂 | 双臂异步任务（如抓蓝莓） |

示例：`test_dual_arm_100hz_follow` 模式 **`both-stream`**。

C：`dual_arm_set_left_joint_target`、`dual_arm_set_right_joint_target`、`dual_arm_start_dual_arm_streaming`、`dual_arm_stop_dual_arm_streaming`、`dual_arm_is_dual_arm_streaming`。

---

## 8. 状态读取

### 8.1 `get_joint`

```cpp
std::tuple<RetCode, std::vector<float>> get_joint(Group group) const;
```

返回当前关节角（rad）。

### 8.2 `get_joint_velocity`

```cpp
std::tuple<RetCode, std::vector<float>> get_joint_velocity(Group group) const;
```

返回当前关节角速度（rad/s）。

### 8.3 `get_group_state`

```cpp
std::tuple<RetCode, std::vector<State>> get_group_state(Group group) const;
```

C：`dual_arm_get_joint`、`dual_arm_get_joint_velocity`、`dual_arm_get_group_state`。

---

## 9. 关节限位与速度

### 9.1 `set_joint_max_speed`

```cpp
std::vector<float> set_joint_max_speed(Group group,
                                       const std::vector<float>& joint_limit);
```

设置关节最大速度（rad/s，取绝对值），下发至 `rt_control`。成功返回原 `limits`；失败返回空向量。

### 9.2 `set_joint_max_acc`（暂未下发）

```cpp
std::vector<float> set_joint_max_acc(Group group,
                                     const std::vector<float>& joint_limit);
```

**当前仅 SDK 本地缓存**，不发送到 `rt_control`。  
加速度限位请在 `config.yaml` → `robot.arm[].safety.joint_soft_limit.acceleration` 配置。

### 9.3 `get_joint_max_limit` / `get_joint_min_limit`

```cpp
std::vector<float> get_joint_max_limit(Group group) const;
std::vector<float> get_joint_min_limit(Group group) const;
```

返回算法软限位（rad）。

---

## 10. 夹爪 API

路径：`SDK → UDP → rt_control → CAN → RMD V4`

`LEFT_ARM` → `gripperL`，`RIGHT_ARM` → `gripperR`。

### 10.1 归一化量纲

| 量 | 范围 | 物理映射 |
|----|------|----------|
| `position` | 0..1000 | 张开(0) .. 闭合(1000) |
| `speed` | 1..1000 | 0.36 .. 360 deg/s |
| `force` | 1..1000 | 0.001 .. 1.0 Nm |

宏定义见 [`cpp/include/gripper_config.h`](../cpp/include/gripper_config.h)。

### 10.2 `set_gripper_config`

```cpp
RetCode set_gripper_config(Group group,
                           uint16_t speed,
                           uint16_t force,
                           bool block,
                           int timeout);
```

| 参数 | 说明 |
|------|------|
| `group` | 仅 `LEFT_ARM` / `RIGHT_ARM` |
| `speed` | 1..1000 |
| `force` | 1..1000 |
| `block` | `true` 阻塞至到位或超时；`false` 非阻塞 |
| `timeout` | 阻塞模式等待秒数（须 > 0） |

### 10.3 `set_gripper_position`

```cpp
RetCode set_gripper_position(Group group,
                             uint16_t position,
                             bool block,
                             int timeout);
```

`position`：0..1000。

### 10.4 `get_gripper_state`

```cpp
std::tuple<RetCode, std::unordered_map<std::string, uint16_t>>
get_gripper_state(Group group) const;
```

返回字段示例：`position`、`force`、`speed`（均为 uint16_t）。

C 结构体：`dual_arm_gripper_state_t { position, force, speed }` + `dual_arm_get_gripper_state`。

---

## 11. API 选用指南

```text
单次点位 / 低跟随规划     → move_joint(follow=false)
高跟随单臂循环            → move_joint(follow=true, ...) 每 ≤10ms
双臂同步 100Hz 单线程     → set_joint_target(L/R) + flush_joint_command
双臂异步独立更新          → start_dual_arm_streaming + set_left/right_joint_target
```

| 模式 | 发包 | 未更新臂行为 | 用户周期要求 |
|------|------|--------------|--------------|
| `move_joint` 单组 | 立即 1 包 | 其他组 feedback hold | 高跟随 ≤10 ms |
| `both-flush` | 用户 flush 1 包 | 未 dirty 臂 feedback hold | 高跟随 ≤10 ms |
| `both-stream` | SDK `@hz` | 左/右 sticky 上次目标 | 仅更新目标，无 flush |

---

## 12. C API 对照表（节选）

| C++ | C |
|-----|---|
| `ControlApi(path)` | `dual_arm_control_api_create(path)` |
| `set_group` | `dual_arm_set_group` |
| `move_joint` | `dual_arm_move_joint` |
| `set_joint_target` | `dual_arm_set_joint_target` |
| `flush_joint_command` | `dual_arm_flush_joint_command` |
| `set_left_joint_target` | `dual_arm_set_left_joint_target` |
| `set_right_joint_target` | `dual_arm_set_right_joint_target` |
| `start_dual_arm_streaming` | `dual_arm_start_dual_arm_streaming` |
| `stop_dual_arm_streaming` | `dual_arm_stop_dual_arm_streaming` |
| `is_dual_arm_streaming` | `dual_arm_is_dual_arm_streaming` |
| `get_joint` | `dual_arm_get_joint` |
| `get_joint_velocity` | `dual_arm_get_joint_velocity` |
| `set_gripper_*` / `get_gripper_state` | `dual_arm_set_gripper_*` / `dual_arm_get_gripper_state` |

---

## 13. 相对草案 docx 的变更摘要

| 项目 | 草案 | 当前实现 |
|------|------|----------|
| 合并双臂发包 | 无 | `set_joint_target` + `flush_joint_command` |
| 独立双臂 streaming | 无 | `set_left/right_joint_target` + `start/stop_dual_arm_streaming` |
| 关节速度 | 无 | `get_joint_velocity` |
| 夹爪量纲 | 范围 `???` | 0..1000 及物理映射已定义 |
| `set_joint_max_acc` | 与 max_speed 类似 | **仅本地缓存，未下发** |
| `State::ERROR` | 草案省略 | 已实现 |
