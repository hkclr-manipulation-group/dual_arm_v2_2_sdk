# SDK 示例程序

所有可执行文件在 **`cpp/build/`** 目录下运行。共享工具见 [`example_common.h`](example_common.h)（配置路径、等待 rt_control、RetCode 名称等）。

## 前置条件

1. 启动 `rt_control`，加载 `cuarm_configuration/dual_v2_2/config.yaml`
2. 编译 SDK：

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
cd cpp/build
```

3. **无需启动 panel** — SDK 通过 UDP 直接向 `rt_control` 发送指令

---

## 快速上手（推荐顺序）

| 顺序 | 程序 | 说明 |
| --- | --- | --- |
| 1 | `./test_control_api` | 交互式冒烟测试，验证通信与基本 API |
| 2 | `./go_home` | 双臂回零（最简单运动示例） |
| 3 | `./move_arms_demo` | 单臂 / 双臂运动，含到位检测与误差报告 |

---

## 入门示例

### `go_home`

双臂 7 关节置 0，头部保持当前角度。

```bash
./go_home
```

### `move_arms_demo`

演示左臂 → 右臂 → 双臂 → 回零。在源码顶部修改 `kLeftOffsetDeg` / `kRightOffsetDeg`（相对当前位姿偏移，单位：度）。

```bash
./move_arms_demo
```

---

## 应用示例

### `teach_replay`

循环回放示教文件中的关节轨迹，Ctrl+C 停止。

```bash
./teach_replay
./teach_replay ../examples/teach_points_left_right.txt
```

**示教数据文件**

| 文件 | 说明 |
| --- | --- |
| `teach_points.txt` | 默认 17 列（含夹爪列） |
| `teach_points_left_right.txt` | 15 列（无夹爪） |
| `teach_points_right.txt` | 仅右臂数据 |

格式（逗号分隔，度）：`idx, left_j1..j7, [left_gripper,] right_j1..j7, [right_gripper]`

---

## 测试 / 高级示例

| 程序 | 用途 | 额外要求 |
| --- | --- | --- |
| `test_control_api` | 组控制、关节读写、低/高跟随、速度限位 | — |
| `test_control_api_various_command_frequency` | 高跟随不同发送频率对比 | — |
| `test_move_joint_curve_fit` | `trajectory_mode=1` 曲线拟合 | — |
| `test_gripper_api` | 单夹爪 API | `config.yaml` 需配置 `robot.gripper` |
| `test_dual_gripper_api` | 双臂夹爪 API | 同上 |
| `test_arm_gripper_safe` | 夹爪 + 臂运动安全流程 | 同上 |

```bash
# 冒烟测试（默认不主动运动；可选 J1 偏移）
./test_control_api
./test_control_api 0.01

# 夹爪测试（需硬件 + 配置）
./test_gripper_api
./test_dual_gripper_api
```

---

## 编写自己的程序

```cpp
#include "control_api.h"

int main() {
    // 路径相对 cpp/build/
    dual_arm_v2_2_sdk::ControlApi api("../../../cuarm_configuration/dual_v2_2");
    // ...
}
```

完整 API 见 [`../include/control_api.h`](../include/control_api.h) 与 [`../../doc/`](../doc/) 接口文档。
