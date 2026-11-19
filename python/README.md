# Dual Arm V2.2 Python

`dual_arm_v2_2_sdk` 是 C++ `ControlApi` 的 pybind11 绑定。接口说明见 [SDK 接口文档](../doc/control_api.md)。

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

## 1. 在 SDK 根目录编译

**下面的编译、打包和安装命令都在 SDK 根目录执行，不是在 `build/` 或 `examples/python/`。** SDK 根目录就是同时有 `CMakeLists.txt` 和 `python/package_wheel.sh` 的目录。如果当前提示符以 `dual_arm_v2_2_sdk_rev/build$` 结尾，先执行 `cd ..`。

仓库不附带预编译的 wheel。建议在目标机器上编译，并用同一个 `python3` 编译、打包、安装和运行。x86_64 与 Jetson（aarch64）的产物不能混用；Python 扩展也要匹配解释器版本，例如 `cpython-310` 对应 CPython 3.10。

依赖：

```bash
sudo apt install -y build-essential cmake python3-dev libyaml-cpp-dev libeigen3-dev
python3 -m pip install pybind11
```

在 SDK 根目录配置并编译：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
      -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON \
      -DDUAL_ARM_V2_2_SDK_BUILD_CPP_EXAMPLES=OFF \
      -DDUAL_ARM_V2_2_SDK_BUILD_C_EXAMPLES=OFF \
      -DPython3_EXECUTABLE="$(which python3)" \
      -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)"
cmake --build build --target dual_arm_v2_2_sdk -j2
```

产物是 `build/python/dual_arm_v2_2_sdk.cpython-<版本>-<架构>-linux-gnu.so`。如果编译时内存不足，改用 `-j1`。

## 2. 选择运行方式

### A. 不安装，直接用本地构建

从 SDK 根目录执行下列命令，仅验证导入路径和错误码，不连接控制器，也不会让机械臂运动：

```bash
PYTHONPATH="$PWD/build/python" python3 -c 'import dual_arm_v2_2_sdk as sdk; print(sdk.__file__); print(sdk.RetCode.TARGET_OUT_OF_RANGE, sdk.RetCode.LINEAR_PATH_FAILED)'
```

输出应指向本仓库的 `build/python/`，而不是已安装的 `site-packages/`。运行 `examples/python/` 下的脚本时使用相同的 `PYTHONPATH` 前缀；如果已经进入 `examples/python/`，改用 `PYTHONPATH=../../build/python`。运动示例会下发指令，不要用它们验证是否加载了新版模块。

### B. 打 wheel 并安装

如果以后想直接 `import dual_arm_v2_2_sdk`，不再设置 `PYTHONPATH`，在 **SDK 根目录**运行：

```bash
python3 -m pip install 'setuptools>=68' build auditwheel patchelf
cmake --build build --target dual_arm_v2_2_sdk -j2 &&
./python/package_wheel.sh 1.0.0 build &&
python3 -m pip install --user --force-reinstall whl_build/wheelhouse/dual_arm_v2_2_sdk-1.0.0-*.whl
```

- `1.0.0` 是 wheel 的版本；`build` 是已经编译好的构建目录，不是输出目录。打包脚本不编译 C++。
- 新 wheel 在 **SDK 根目录的 `whl_build/wheelhouse/`**，不在 `build/whl_build/`。平台后缀由脚本生成，命令里的 `*` 不要加引号或反斜杠。
- 改过 SDK 或 Python 绑定后，重复上面的「编译 → 打包 → 安装」。如果版本仍是 `1.0.0`，`--force-reinstall` 可确保替换旧包。在虚拟环境中安装时去掉 `--user`。

安装后，不带 `PYTHONPATH` 验证：

```bash
env -u PYTHONPATH python3 -c 'import dual_arm_v2_2_sdk as sdk; print(sdk.__file__); print(sdk.RetCode.TARGET_OUT_OF_RANGE, sdk.RetCode.LINEAR_PATH_FAILED)'
```

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `python/package_wheel.sh: No such file or directory` | 当前目录通常是 `build/`。回到 SDK 根目录再执行脚本。 |
| `missing build/python` 或找不到扩展 `.so` | 确认在 SDK 根目录，已启用 `DUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON` 并完成编译。 |
| `auditwheel` 或 `patchelf: command not found` | 确认工具已安装，且工具所在目录在 `PATH` 中；用户级安装通常使用 `$HOME/.local/bin`。 |
| `pip` 找不到 `.whl` 文件 | 先看打包是否成功，再从 SDK 根目录检查 `whl_build/wheelhouse/`。不要用上次残留的旧 wheel。 |
| 导入后只有 `RetCode.TIMEOUT` 等旧错误码 | 用上面的导入命令检查实际加载位置，确认新版已经重编译或重新安装。 |
