# Dual Arm V2.2 控制 SDK

本仓库提供 **Dual Arm V2.2 的 C++ 客户端控制 API**。SDK 通过 UDP 与实时控制服务 `rt_control` 通信，向机器人下发运动与夹爪指令，并接收关节/夹爪状态反馈。

> **重要说明**：本仓库**不能单独运行**。它只包含客户端 SDK 与示例程序；实际驱动机器人还需要 **`cuarm_rt_control`**（生成 `dual_v2_2_rt_control` 可执行文件）和 **`cuarm_configuration`**（机器人配置与 URDF）。编译 SDK 本身还依赖同级的 **`robot_platform_utils`** 与 **`curi_udp`**。

接口定义见 [`cpp/include/control_api.h`](cpp/include/control_api.h)。

---

## 仓库定位

| 组件 | 是否包含在本仓库 | 作用 |
| --- | --- | --- |
| `dual_arm_v2_2_sdk`（本仓库） | ✅ | C++ 控制 API、示例程序 |
| `cuarm_rt_control` | ❌ 需另行获取 | 实时控制服务（`dual_v2_2_rt_control`） |
| `cuarm_configuration` | ❌ 需另行获取 | 机器人参数、URDF、网络端口等 |
| `robot_platform_utils` | ❌ 需另行获取 | 配置加载、UDP 消息协议（编译依赖） |
| `curi_udp` | ❌ 需另行获取 | UDP 通信库（编译依赖） |
| `cuarm_upper_software`（panel） | ❌ 不需要 | 图形操作面板；**使用 SDK 时无需启动** |

数据流：

```text
你的程序 (SDK) ──UDP──▶ dual_v2_2_rt_control ──CAN/硬件──▶ 双臂 + 夹爪
```

---

## 依赖一览

### 编译 SDK 所需（C++ 库 + 示例）

| 依赖 | 版本建议 | 说明 |
| --- | --- | --- |
| [CMake](https://cmake.org/download/) | ≥ 3.16 | 构建系统 |
| C++ 编译器 | C++17 | GCC / Clang |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.8.0 | 读取 `config.yaml` |
| [robot_platform_utils](https://github.com/hkclr-manipulation-group/robot_platform_utils) | — | 须与本仓库处于**同一工作区根目录** |
| [curi_udp](https://github.com/CURI-Simulation-and-Software-Group/curi_udp) | — | 同上，须为同级目录 |

Ubuntu 快速安装 yaml-cpp：

```bash
sudo apt-get install libyaml-cpp-dev
# 或从源码安装，见 robot_platform_utils/README.md
```

### 运行示例 / 集成所需（运行时）

| 依赖 | 说明 |
| --- | --- |
| [cuarm_rt_control](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control) | 编译产物 `dual_v2_2_rt_control`；详见其 README 与 `cuarm_robotics/README.md` |
| [cuarm_configuration](https://github.com/CURI-Simulation-and-Software-Group/cuarm_configuration) | 使用 `dual_v2_2/` 配置目录 |
| 机器人硬件或仿真 | 由 `config.yaml` 中 `rt_control.hardware_simulation` 控制 |

### 编译 rt_control 所需（构建 `dual_v2_2_rt_control`）

| 依赖 | 说明 |
| --- | --- |
| Pinocchio、Eigen3、Boost、urdfdom 等 | 详见 [`cuarm_robotics/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/cuarm_robotics/README.md) |
| `cuarm_robotics` | `cuarm_rt_control` 子模块 |
| `robot_platform_utils`、`curi_udp`/`curi_tcp` | 与工作区根目录同级 |
| `gripper/` 目录 | **编译时必需**（CMake 固定引用，与运行时是否启用夹爪无关） |

`gripper/` 须包含以下三个子目录：

| 子目录 | 来源 |
| --- | --- |
| `gripper/cxt_can/` | [hkclr-manipulation-group/cxt_can](https://github.com/hkclr-manipulation-group/cxt_can) |
| `gripper/rmd_motor/` | [hkclr-manipulation-group/rmd_motor](https://github.com/hkclr-manipulation-group/rmd_motor) |
| `gripper/temp_spark2_gripper/` | [hkclr-manipulation-group/temp_spark2_gripper](https://github.com/hkclr-manipulation-group/temp_spark2_gripper) |

> **编译 vs 运行**：即使配置为 `no_gripper.yaml`，编译 `dual_v2_2_rt_control` 仍需要 `gripper/` 目录；运行时只有配置了 `robot.gripper` 且 `hardware_simulation: false` 时才会连接 CAN 夹爪硬件。

---

## 工作区目录布局

`cpp/CMakeLists.txt` 将 SDK 上两级目录（`../..`）视为**工作区根目录**，并在该根目录下查找 `robot_platform_utils` 与 `curi_udp`。请按如下结构组织：

```text
<workspace>/
├── dual_arm_v2_2_sdk/          # 本仓库（clone 后目录名可自定，但需保持相对位置）
│   ├── cpp/
│   ├── doc/
│   └── README.md
├── robot_platform_utils/       # 编译依赖
├── curi_udp/                   # 编译依赖
├── cuarm_configuration/        # 运行时配置
│   └── dual_v2_2/
│       └── config.yaml
├── cuarm_rt_control/           # 编译并运行 rt_control
│   └── build/
│       └── dual_v2_2_rt_control
└── gripper/                    # 编译 rt_control 时必需（运行时启用夹爪才需 CAN 硬件）
    ├── cxt_can/
    ├── rmd_motor/
    └── temp_spark2_gripper/
```

### 推荐方式：克隆 monorepo

若希望一次性获取全部相关仓库，可克隆 [`cuarm_panel_control`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_panel_control) monorepo（含 submodule），再将本 SDK 放入同级或替换其中的 `dual_arm_v2_2_sdk` 目录：

```bash
git clone --recurse-submodules git@github.com:CURI-Simulation-and-Software-Group/cuarm_panel_control.git
cd cuarm_panel_control

# 若单独分发本 SDK，可在此目录下 clone：
git clone git@github.com:hkclr-manipulation-group/dual_arm_v2_2_sdk.git
```

### 最小手动组装

仅使用 SDK 时，至少 clone 以下仓库到同一 `<workspace>/` 下：

```bash
mkdir -p ~/cuarm_workspace && cd ~/cuarm_workspace

git clone git@github.com:hkclr-manipulation-group/dual_arm_v2_2_sdk.git
git clone git@github.com:hkclr-manipulation-group/robot_platform_utils.git
git clone git@github.com:CURI-Simulation-and-Software-Group/curi_udp.git
git clone git@github.com:CURI-Simulation-and-Software-Group/cuarm_configuration.git
git clone --recurse-submodules git@github.com:CURI-Simulation-and-Software-Group/cuarm_rt_control.git

# 编译 rt_control 还需要 gripper/ 目录（见上文「编译 rt_control 所需」）
mkdir -p gripper && cd gripper
git clone https://github.com/hkclr-manipulation-group/cxt_can.git
git clone https://github.com/hkclr-manipulation-group/rmd_motor.git
git clone https://github.com/hkclr-manipulation-group/temp_spark2_gripper.git
```

构建 `rt_control` 请参考 [`cuarm_rt_control/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/README.md) 与 [`cuarm_robotics/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/cuarm_robotics/README.md)。

---

## 快速开始

### 1. 配置

编辑 `<workspace>/cuarm_configuration/dual_v2_2/config.yaml`：

- **无夹爪**：`robot.import_yaml: share/config/assembly/no_gripper.yaml`，不写 `robot.gripper`
- **有夹爪**：配置 `robot.gripper` 并确保 CAN 硬件在线

确认 `panel.ip:port` 与 `rt_control.ip:port` 与运行环境一致（默认均为 `127.0.0.1`，端口 `11101` / `11201`）。

### 2. 启动 rt_control

```bash
cd <workspace>/cuarm_rt_control/build
sudo ./dual_v2_2_rt_control
# 或显式指定配置目录：
sudo ./dual_v2_2_rt_control /path/to/cuarm_configuration/dual_v2_2
```

### 3. 编译并运行 SDK 示例

```bash
cd <workspace>/dual_arm_v2_2_sdk
cmake -S cpp -B cpp/build
cmake --build cpp/build -j
cd cpp/build

# 推荐第一步：验证通信
./test_control_api

# 简单运动
./go_home
./move_arms_demo
```

示例程序默认配置路径为 `../../../cuarm_configuration/dual_v2_2`（相对于 `cpp/build/` 运行）。若目录布局不同，请修改 [`cpp/examples/example_common.h`](cpp/examples/example_common.h) 中的 `kConfigPath`，或在集成时传入绝对路径。

**无需启动 panel** — SDK 绑定 `panel.ip:port` 接收状态，向 `rt_control.ip:port` 发送指令。

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
- rt_control 构建：[`cuarm_rt_control`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control)
- 公共工具库：[`robot_platform_utils`](https://github.com/hkclr-manipulation-group/robot_platform_utils)

---

## 集成到自己的项目

```cpp
#include "control_api.h"

dual_arm_v2_2_sdk::ControlApi api("/path/to/cuarm_configuration/dual_v2_2");

// 等待 rt_control
std::vector<float> joints;
// ... poll api.get_joint() until SUCCESS ...

api.set_group(dual_arm_v2_2_sdk::Group::LEFT_ARM, true);
api.move_joint(dual_arm_v2_2_sdk::Group::LEFT_ARM, target_rad, false);
```

CMake 集成要点：

1. 链接 `dual_arm_v2_2_control_sdk` 静态库（或直接引用其源文件，见 [`cpp/CMakeLists.txt`](cpp/CMakeLists.txt)）
2. 包含目录：`cpp/include`、工作区根目录、`robot_platform_utils/cpp/include`、`curi_udp/c`
3. 链接 `Threads::Threads` 与 `yaml-cpp::yaml-cpp`
4. 确保 `robot_platform_utils` 与 `curi_udp` 与本仓库处于预期的同级目录，或自行调整 `REPOSITORY_ROOT` 逻辑
