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

## 从 git 编译

仓库里没有预编译的 wheel。`whl_build/` 在 `.gitignore` 中，克隆之后需要在**要运行的那台机器**上自己编译。x86_64 上编出的 `.so` 不能拿到 Jetson（aarch64）上用，反过来也不行；在 Jetson 上执行下面同一组命令即可，没有单独的 ARM 开关。扩展还绑定编译时使用的 Python 版本，JetPack 6 一般是 3.10，JetPack 5 一般是 3.8。

依赖：

```bash
sudo apt install -y build-essential cmake python3-dev libyaml-cpp-dev libeigen3-dev
python3 -m pip install pybind11
```

`pybind11` 未安装且机器能访问 GitHub 时，CMake 会自行拉取 v2.13.6。在仓库根目录编译：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON \
      -DDUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES=OFF \
      -DDUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES=OFF \
      -DPython3_EXECUTABLE="$(which python3)" \
      -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)"
cmake --build build -j
```

`control_api.cpp` 较大，Jetson 内存紧张时把 `-j` 降到 4。编完不必安装，把编译目录加到 `PYTHONPATH`：

```bash
PYTHONPATH=build/python python3 python/smoke_test.py
PYTHONPATH=build/python python3 examples/python/teach.py
```

产物是 `build/python/dual_arm_v2_2_sdk.cpython-<版本>-<架构>-linux-gnu.so`。

若希望之后不用再写 `PYTHONPATH`，在本机打 wheel 再安装。打包脚本还需要 `build`、`auditwheel` 和 `patchelf`。wheel 文件名随平台变化，不要写死某一份 `manylinux_*_x86_64.whl`。

```bash
python3 -m pip install build auditwheel patchelf
python/package_wheel.sh 1.0.0 build
python3 -m pip install --user whl_build/wheelhouse/dual_arm_v2_2_sdk-1.0.0-*.whl
```

示例在 `examples/python/`。改过 C++ 或绑定后需要重新编译；已经 `pip install` 过的，还要再安装一次新 wheel。
