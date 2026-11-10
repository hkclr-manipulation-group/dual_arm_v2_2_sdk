# Dual Arm V2.2 Python

`dual_arm_v2_2_sdk` 是 C++ `ControlApi` 的 pybind11 绑定。方法名、参数和返回值与 C++ 相同，说明见 SDK 文档 `doc/control_api.md`。

```python
import dual_arm_v2_2_sdk as sdk

api = sdk.ControlApi("/path/to/cuarm_configuration/dual_v2_2")
api.set_group(sdk.Group.LEFT_ARM, True)
code, joints = api.get_joint(sdk.Group.LEFT_ARM)
```

- 构造参数是含 `config.yaml` 的配置目录。
- 查询接口返回 `(RetCode, list)`；夹爪状态返回 `(RetCode, dict)`。
- 关节单位为弧度。末端位姿每臂 7 个数：`x, y, z, qw, qx, qy, qz`。
- `set_joint_max_acc` 未导出。加速度写在 `config.yaml` 的 `robot.arm[].safety.joint_soft_limit.acceleration`。

从源码打 wheel：

```bash
cmake -S . -B build_wheel -DCMAKE_BUILD_TYPE=Release \
      -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON \
      -DDUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES=OFF \
      -DDUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES=OFF
cmake --build build_wheel -j
python/package_wheel.sh
```

开发时不必安装 wheel，把编译产物放到 `PYTHONPATH` 即可：

```bash
PYTHONPATH=build/python python3 python/smoke_test.py
```

示例在 `examples/python/`。
