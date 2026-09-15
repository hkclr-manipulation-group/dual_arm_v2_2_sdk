# Dual Arm V2.2 控制 SDK

本仓库提供 **Dual Arm V2.2 的 C++ 客户端控制 API**。SDK 通过 UDP 与实时控制服务 `rt_control` 通信，向机器人下发运动与夹爪指令，并接收关节/夹爪状态反馈。

> **重要说明**：本仓库包含客户端 SDK、示例程序与 **`dual_v2_2` 运行时配置**（`cuarm_configuration/`）。实际驱动机器人还需要 **`cuarm_rt_control`**（生成 `dual_v2_2_rt_control` 可执行文件）。编译 SDK 依赖本仓库内的 **`robot_platform_utils`** 与 **`curi_udp`** 子模块。

接口定义见 [`cpp/include/control_api.h`](cpp/include/control_api.h)。

---

## 仓库定位

| 组件 | 是否包含在本仓库 | 作用 |
| --- | --- | --- |
| `dual_arm_v2_2_sdk`（本仓库） | ✅ | C++ 控制 API、示例程序、`cuarm_configuration/dual_v2_2` 配置 |
| `cuarm_rt_control` | ❌ 需另行获取 | 实时控制服务（`dual_v2_2_rt_control`） |
| `robot_platform_utils` | ✅ 子模块 | 配置加载、UDP 消息协议（编译依赖） |
| `curi_udp` | ✅ 子模块 | UDP 通信库（编译依赖） |
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
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 系统包即可 | **仅编译本 SDK 时需要**；链入 `libdual_arm_v2_2_control_sdk.so`，上层集成方无需安装 dev 包 |
| [robot_platform_utils](https://github.com/hkclr-manipulation-group/robot_platform_utils) | — | 本仓库子模块 |
| [curi_udp](https://github.com/CURI-Simulation-and-Software-Group/curi_udp) | — | 本仓库子模块 |

编译本仓库时：

```bash
sudo apt-get install libyaml-cpp-dev
```

### 运行示例 / 集成所需（运行时）

| 依赖 | 说明 |
| --- | --- |
| [cuarm_rt_control](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control) | 编译产物 `dual_v2_2_rt_control`；详见其 README 与 `cuarm_robotics/README.md` |
| 本仓库 `cuarm_configuration/dual_v2_2/` | 机器人参数、URDF、网络端口等 |
| 机器人硬件或仿真 | 由 `config.yaml` 中 `rt_control.hardware_simulation` 控制 |

### 编译 rt_control 所需（构建 `dual_v2_2_rt_control`）

| 依赖 | 说明 |
| --- | --- |
| Pinocchio、Eigen3、Boost、urdfdom 等 | 详见 [`cuarm_robotics/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/cuarm_robotics/README.md) |
| `cuarm_robotics` | `cuarm_rt_control` 子模块 |
| `robot_platform_utils`、`curi_udp`/`curi_tcp` | 通常与 `cuarm_rt_control` 同级；本 SDK 已自带对应子模块 |
| `gripper/` 目录 | **编译时必需**（CMake 固定引用，与运行时是否启用夹爪无关） |

`gripper/` 须包含以下三个子目录：

| 子目录 | 来源 |
| --- | --- |
| `gripper/cxt_can/` | [hkclr-manipulation-group/cxt_can](https://github.com/hkclr-manipulation-group/cxt_can) |
| `gripper/rmd_motor/` | [hkclr-manipulation-group/rmd_motor](https://github.com/hkclr-manipulation-group/rmd_motor) |
| `gripper/temp_spark2_gripper/` | [hkclr-manipulation-group/temp_spark2_gripper](https://github.com/hkclr-manipulation-group/temp_spark2_gripper) |

> **编译 vs 运行**：即使配置为 `no_gripper.yaml`，编译 `dual_v2_2_rt_control` 仍需要 `gripper/` 目录；运行时只有配置了 `robot.gripper` 且 `hardware_simulation: false` 时才会连接 CAN 夹爪硬件。

---

## 仓库目录布局

```text
dual_arm_v2_2_sdk/
├── CMakeLists.txt              # 构建入口（SDK + 示例）
├── cpp/                        # SDK 库源码与头文件
│   ├── include/
│   └── src/
├── examples/                   # 示例程序
├── cuarm_configuration/        # 运行时配置
│   └── dual_v2_2/
│       └── config.yaml
├── robot_platform_utils/       # 子模块（编译依赖）
├── curi_udp/                   # 子模块（编译依赖）
├── doc/
└── build/                      # 构建产物（out-of-source）
    ├── bin/                    # 示例可执行文件
    └── lib/                    # libdual_arm_v2_2_control_sdk.so
```

`rt_control` 仍须单独获取并编译，例如与 SDK 放在同一工作区：

```text
<workspace>/
├── dual_arm_v2_2_sdk/          # 本仓库
└── cuarm_rt_control/           # 编译并运行 rt_control
    └── build/
        └── dual_v2_2_rt_control
```

克隆本仓库时请带上子模块：

```bash
git clone --recurse-submodules git@github.com:hkclr-manipulation-group/dual_arm_v2_2_sdk.git
# 或已 clone 后：
git submodule update --init --recursive
```

构建 `rt_control` 请参考 [`cuarm_rt_control/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/README.md) 与 [`cuarm_robotics/README.md`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control/blob/main/cuarm_robotics/README.md)。

---

## 快速开始

### 1. 配置

编辑本仓库 `cuarm_configuration/dual_v2_2/config.yaml`：

- **无夹爪**：`robot.import_yaml: share/config/assembly/no_gripper.yaml`，不写 `robot.gripper`
- **有夹爪**：配置 `robot.gripper` 并确保 CAN 硬件在线

确认 `panel.ip:port` 与 `rt_control.ip:port` 与运行环境一致（默认均为 `127.0.0.1`，端口 `11101` / `11201`）。

### 2. 启动 rt_control

```bash
cd <workspace>/cuarm_rt_control/build
sudo ./dual_v2_2_rt_control
# 或显式指定配置目录：
sudo ./dual_v2_2_rt_control /path/to/dual_arm_v2_2_sdk/cuarm_configuration/dual_v2_2
```

### 3. 编译并运行 SDK 示例

```bash
cd dual_arm_v2_2_sdk
cmake -S . -B build
cmake --build build -j

# 在 build/ 或 build/bin/ 下运行均可
cd build
./bin/test_control_api    # 推荐第一步：验证通信
./bin/go_home             # 双臂回零
./bin/move_arms_demo      # 单臂/双臂运动演示

# 等价写法
cd build/bin
./test_control_api
./go_home
./move_arms_demo
```

构建说明：

- **禁止源码内构建**：请勿在仓库根目录直接 `cmake .`；始终使用 `-S . -B build`。
- **关闭示例**（仅编译 SDK 库）：`cmake -S . -B build -DDUAL_ARM_V2_2_SDK_BUILD_EXAMPLES=OFF`
- **默认安装前缀**：`build/install`（`cmake --install build`）

示例程序通过 [`examples/example_common.h`](examples/example_common.h) 中的 `default_config_path()` **按可执行文件位置**自动查找 `cuarm_configuration/dual_v2_2`，因此从 `build/` 或 `build/bin/` 启动都能找到配置。自定义布局时，可修改该函数，或在集成时向 `ControlApi` 传入绝对路径。

**无需启动 panel** — SDK 绑定 `panel.ip:port` 接收状态，向 `rt_control.ip:port` 发送指令。

更多示例说明见 **[`examples/README.md`](examples/README.md)**。

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
| position | 0..1000 | 张开..闭合 |
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
- 示例详解：[`examples/README.md`](examples/README.md)
- rt_control 构建：[`cuarm_rt_control`](https://github.com/CURI-Simulation-and-Software-Group/cuarm_rt_control)
- 公共工具库：[`robot_platform_utils`](https://github.com/hkclr-manipulation-group/robot_platform_utils)

---

## 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `OS failed to open input stream handle` / `bad conversion` | 找不到 `config.yaml` | 确认 `cuarm_configuration/dual_v2_2/config.yaml` 存在；集成时使用绝对路径 |
| `No state received within 3 seconds` | `rt_control` 未启动或 UDP 地址/端口不匹配 | 启动 `dual_v2_2_rt_control`，并核对 `config.yaml` 中 `panel` / `rt_control` 的 `ip:port` |
| 找不到 `libdual_arm_v2_2_control_sdk.so` | 动态库不在搜索路径 | 设置 `LD_LIBRARY_PATH` 指向 `build/lib` 或安装目录下的 `lib/` |

---

## 集成到自己的项目

先安装 SDK（动态库 + 头文件 + CMake 包）：

```bash
cd dual_arm_v2_2_sdk
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$HOME/.local
cmake --build build -j
cmake --install build
```

若前缀不是系统默认路径，在自己的工程里带上 `CMAKE_PREFIX_PATH`（或设置 `dual_arm_v2_2_control_sdk_DIR`）。

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_app LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(dual_arm_v2_2_control_sdk REQUIRED)
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE dual_arm_v2_2_control_sdk::dual_arm_v2_2_control_sdk)
```

```cpp
#include "control_api.h"

dual_arm_v2_2_sdk::ControlApi api("/path/to/dual_arm_v2_2_sdk/cuarm_configuration/dual_v2_2");

// 等待 rt_control
std::vector<float> joints;
// ... poll api.get_joint() until SUCCESS ...

api.set_group(dual_arm_v2_2_sdk::Group::LEFT_ARM, true);
api.move_joint(dual_arm_v2_2_sdk::Group::LEFT_ARM, target_rad, false);
```

`find_package` 只会带上 `Threads`。yaml-cpp 在编译 SDK 时已 **PRIVATE** 链入 `libdual_arm_v2_2_control_sdk.so`，上层工程不必安装或链接 yaml-cpp。部署时把 `lib/` 加入 `LD_LIBRARY_PATH`（或 `ldconfig`）即可；Ubuntu 上通常会把 `libyaml-cpp.a` 静态编进 SDK，运行时不再单独依赖 `libyaml-cpp.so`。

不安装、直接把本仓库 `add_subdirectory` 也可以：

```cmake
add_subdirectory(path/to/dual_arm_v2_2_sdk)
target_link_libraries(my_app PRIVATE dual_arm_v2_2_control_sdk)
# 可选：-DDUAL_ARM_V2_2_SDK_BUILD_EXAMPLES=OFF
```