# SDK 示例程序

C++ 与 C 示例分别位于 **`examples/cpp/`** 与 **`examples/c/`**，Python 示例在 **`examples/python/`**，示教数据在 **`examples/data/`**。

可执行文件输出到：

- C++：`build/bin/cpp/`（如 `./bin/cpp/go_home`）
- C：`build/bin/c/`（如 `./bin/c/go_home`）

共享工具：

- C++：[`examples/cpp/example_common.h`](cpp/example_common.h)
- C：[`examples/c/example_common.h`](c/example_common.h)、[`example_velocity_guard.h`](c/example_velocity_guard.h)

## 前置条件

1. 启动 `rt_control`，加载本仓库 `cuarm_configuration/dual_v2_2/config.yaml`
2. 编译 SDK：

```bash
cmake -S . -B build
cmake --build build -j
cd build
export LD_LIBRARY_PATH=$PWD/lib:$LD_LIBRARY_PATH
```

3. **无需启动 panel**

关闭示例：

```bash
cmake -S . -B build -DDUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES=OFF
cmake -S . -B build -DDUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES=OFF
```

---

## 示例一览

| 程序 | C++ | C | 说明 |
| --- | --- | --- | --- |
| 回零 | `./bin/cpp/go_home` | `./bin/c/go_home` | 双臂回零 |
| 运动演示 | `move_arms_demo` | 同左 | 单臂/双臂运动 |
| 示教回放 | `teach_replay` | 同左 | 循环回放示教文件 |
| 冒烟测试 | `test_control_api` | 同左 | 交互式 API 测试 |
| 夹爪 | `test_gripper_api` | 同左 | 左夹爪 API |
| 100Hz 跟随 | `test_dual_arm_100hz_follow` | 同左 | 高跟随模式对比 |
| 低跟随独立 MoveL | `dual_arm_independent_move_line` | — | `set_*_tool_line_target` + `start_dual_arm_tool_line_auto_flush`（合包 ack，非 ALL/14） |
| 顺序双臂 MoveL | `test_dual_arm_sequential_move_line` | — | 左 out/back → 右 out/back，每臂 `move_line`（旧独立测试行为） |
| 错峰双臂流式 | — | `staggered_dual_arm_stream` | 左先动→右再动→左先回零→右再回零 |

完整列表见根目录 `CMakeLists.txt` 中的 `_dual_arm_shared_examples` 与 `_dual_arm_cpp_only_examples`。

---

## 示教文件

默认回放 [`examples/data/teach_points_left_right.txt`](data/teach_points_left_right.txt)：

```bash
./bin/cpp/teach_replay
./bin/cpp/teach_replay ../../examples/data/teach_points.txt
./bin/c/teach_replay ../../examples/data/teach_points_left_right.txt
```

---

## 配置路径

示例会从 `build/bin/cpp` 或 `build/bin/c` 自动查找 `../../../cuarm_configuration/dual_v2_2`。

也可显式指定：

```bash
export DUAL_ARM_CONFIG_PREFIX=/path/to/cuarm_configuration/dual_v2_2
./bin/c/test_gripper_api
```

C++ 示例还支持命令行首参传入配置目录（见 `config_path_from_args`）。

---

## Python

先打开 Python 绑定再编译（见 [`python/README.md`](../python/README.md)）。示例与 C++ 一样接收配置目录作为首参，也会读 `DUAL_ARM_CONFIG_PREFIX`。

```bash
PYTHONPATH=build/python python3 examples/python/go_home.py
PYTHONPATH=build/python python3 examples/python/move_line.py
PYTHONPATH=build/python python3 examples/python/teach.py
PYTHONPATH=build/python python3 examples/python/test_move_joint_speed.py
PYTHONPATH=build/python python3 examples/python/dual_two_control_api.py --left movel --right hold
PYTHONPATH=build/python python3 examples/python/dual_two_control_sequence.py
```

| 脚本 | 说明 |
| --- | --- |
| `go_home.py` | 双臂回零，头部保持当前角度 |
| `move_line.py` | 左臂末端 z 增加 5 cm 后返回 |
| `teach.py` | 重力补偿示教，输入 `s` 记录关节角 |
| `test_move_joint_speed.py` | 单臂按多个 `speed_ratio` 走到可调目标角再返回 |
| `dual_two_control_api.py` | 两个 ControlApi 并行测试 MoveJ / MoveL / 静止，与 `dual_two_control_api.cpp` 同一套参数 |
| `dual_two_control_sequence.py` | 按固定九步依次执行左右臂 MoveJ / MoveL / 静止组合 |

---

## API 头文件

- C++：[`cpp/include/control_api.h`](../cpp/include/control_api.h)
- C：[`c/include/dual_arm_control_api.h`](../c/include/dual_arm_control_api.h)
