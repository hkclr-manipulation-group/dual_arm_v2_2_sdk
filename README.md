# Dual Arm V2.2 控制 SDK

C++ 控制 API，通过 UDP 与 `rt_control` 通信。接口定义见 [`cpp/include/control_api.h`](cpp/include/control_api.h)。

## 快速开始

### 1. 配置

编辑 [`cuarm_configuration/dual_v2_2/config.yaml`](../cuarm_configuration/dual_v2_2/config.yaml)：

- 无夹爪：`robot.import_yaml: share/config/assembly/no_gripper.yaml`，不写 `robot.gripper`
- 有夹爪：配置 `robot.gripper` 并确保 CAN 硬件在线

### 2. 启动 rt_control

```bash
# 在 rt_control 可执行文件目录下，指定配置路径
./dual_v2_2_rt_control /path/to/cuarm_configuration/dual_v2_2
```

### 3. 编译并运行示例

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
cd cpp/build

# 推荐第一步：验证通信
./test_control_api

# 简单运动
./go_home
./move_arms_demo
```

**无需启动 panel** — SDK 绑定 `panel.ip:port`，向 `rt_control.ip:port` 发指令。

更多示例说明见 **[`cpp/examples/README.md`](cpp/examples/README.md)**。

---

## API 概览

### `move_joint` 模式映射

| SDK 模式 | rt_control 目标模式 | 插值 |
| --- | --- | --- |
| `follow=false` | `kSinglePoint` | `kQuintic`（低跟随） |
| `follow=true, trajectory_mode=0` | `kSinglePoint` | `kDirect`（完全透传） |
| `follow=true, trajectory_mode=1` | `kSinglePoint` | `kQuintic`（曲线拟合） |
| `follow=true, trajectory_mode=2` | `kRawSinglePoint` | `kNone`（二阶低通滤波） |

滤波模式 `radio` 对数映射截止频率：

```text
cutoff_hz = 20 * (1 / 20)^(radio / 999)
```

`radio=0` → 20 Hz，`radio=500` → ~4.47 Hz，`radio=999` → 1 Hz。

### 关节组顺序

`Group::ALL` 向量顺序：**左臂(7) + 头部(2) + 右臂(7)**。  
`Group::ELEVATOR` 暂不支持。

### 夹爪 API

路径：`SDK → UDP → rt_control → CXT CAN → RMD V4`

| 量 | 范围 | 物理含义 |
| --- | --- | --- |
| position | 0..1000 | 闭合..张开 |
| speed | 1..1000 | 0.36..360 deg/s |
| force | 1..1000 | 0.001..1.0 Nm |

宏定义见 [`cpp/include/gripper_config.h`](cpp/include/gripper_config.h)。  
`LEFT_ARM` / `RIGHT_ARM` 分别对应左/右夹爪。

### 暂未实现的接口

- **`set_joint_max_acc`**：当前仅 SDK 本地缓存，不下发 `rt_control`。加速度限位请在 `config.yaml` 的 `robot.arm[].safety.joint_soft_limit.acceleration` 中配置。

---

## 示例程序一览

| 类别 | 程序 | 说明 |
| --- | --- | --- |
| 入门 | `go_home` | 双臂回零 |
| 入门 | `move_arms_demo` | 单臂/双臂运动演示 |
| 应用 | `teach_replay` | 示教点循环回放 |
| 测试 | `test_control_api` | 交互式 API 冒烟测试 |
| 测试 | `test_move_joint_curve_fit` | 曲线拟合模式 |
| 测试 | `test_gripper_api` / `test_dual_gripper_api` | 夹爪 API |
| 测试 | `test_arm_gripper_safe` | 臂+夹爪安全流程 |
| 测试 | `test_control_api_various_command_frequency` | 高跟随频率对比 |

---

## 文档

- 接口草案：[`doc/control_api_draft_20260902v1.pdf`](doc/control_api_draft_20260902v1.pdf)
- 示例详解：[`cpp/examples/README.md`](cpp/examples/README.md)

---

## 集成到自己的项目

```cpp
#include "control_api.h"

dual_arm_v2_2_sdk::ControlApi api("path/to/cuarm_configuration/dual_v2_2");

// 等待 rt_control
std::vector<float> joints;
// ... poll api.get_joint() until SUCCESS ...

api.set_group(dual_arm_v2_2_sdk::Group::LEFT_ARM, true);
api.move_joint(dual_arm_v2_2_sdk::Group::LEFT_ARM, target_rad, false);
```

链接 `dual_arm_v2_2_control_sdk` 库，并包含 `cpp/include` 与仓库根目录（用于 `robot_platform_utils` 头文件）。
