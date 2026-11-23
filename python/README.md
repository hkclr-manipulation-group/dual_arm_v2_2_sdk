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

## 1. 从源码一次构建 C、C++、Python 和 wheel

在含 `CMakeLists.txt` 和 `python/package_wheel.sh` 的 SDK 根目录执行。首次检出时初始化子模块，安装构建工具。以下命令适用于 Ubuntu / Debian。

```bash
git submodule update --init --recursive
sudo apt install -y build-essential cmake binutils libyaml-cpp-dev libeigen3-dev python3-dev python3-venv
python3 -m venv build/venv
source build/venv/bin/activate
python3 -m pip install pybind11 'setuptools>=68' wheel build auditwheel patchelf
```

新建构建目录时，CMake 默认开启 Python 绑定和 C / C++ 示例。在 `build/` 中配置并编译，再回到 SDK 根目录打包。

```bash
mkdir -p build &&
cd build &&
cmake .. &&
make -j4 &&
cd .. &&
./python/package_wheel.sh 1.0.0 build/
```

| 产物 | 路径 |
| --- | --- |
| C / C++ 动态库 | `build/lib/` |
| C / C++ 示例 | `build/bin/c/`、`build/bin/cpp/` |
| Python 扩展 | `build/python/` |
| 可安装的 wheel | `whl_build/wheelhouse/` |

内存不足时用 `make -j2` 或 `make -j1`。C / C++ 动态库、头文件和 CMake 包可在 `build/` 中用 `make install` 安装到 `build/install/`。

如果旧构建目录缓存了 `DUAL_ARM_V2_2_SDK_BUILD_PYTHON=OFF`，在该目录执行一次 `cmake .. -DDUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON`。需要 Release 优化时，配置命令可加 `-DCMAKE_BUILD_TYPE=Release`。

## 2. 安装 wheel 后直接运行脚本

在已激活的虚拟环境中安装并做离线验证：

```bash
python3 -m pip install --force-reinstall whl_build/wheelhouse/dual_arm_v2_2_sdk-1.0.0-*.whl
python3 -c 'import dual_arm_v2_2_sdk as sdk; print(sdk.__file__, sdk.__version__)'
python3 examples/python/test_move_joint_speed.py --help
```

分发时把 `whl_build/wheelhouse/` 里的 `.whl` 交给用户。用户在自己的 Python 环境里执行 `python3 -m pip install /path/to/文件名.whl`，之后用同一个 `python3` 运行脚本，无需设置 `PYTHONPATH`。新终端中需重新激活对应的虚拟环境。

wheel 的标签限定了 CPython 版本、CPU 架构和 Linux 兼容范围。比如 `cp310-cp310-manylinux_2_35_x86_64` 对应 CPython 3.10、x86_64 和 glibc 2.35 或更新的兼容系统。Jetson（aarch64）和其他 CPython 版本需要各自的 wheel。

使用机器人时，还需另行提供含 `config.yaml` 的配置目录并启动 `rt_control`。如果要分发仓库示例，请一并交付 `examples/python/` 和所用的 `cuarm_configuration/dual_v2_2/`。

### 从构建目录临时导入

调试时可直接从构建目录导入 Python 扩展：

```bash
PYTHONPATH="$PWD/build/python" python3 -c 'import dual_arm_v2_2_sdk as sdk; print(sdk.__file__)'
```

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `python/package_wheel.sh: No such file or directory` | 当前目录通常是 `build/`。回到 SDK 根目录再执行脚本。 |
| 找不到 `build/python` 或匹配的扩展 `.so` | 确认已启用 `DUAL_ARM_V2_2_SDK_BUILD_PYTHON=ON`，并使用同一 `python3` 构建和打包。 |
| `auditwheel` 或 `patchelf: command not found` | 激活安装了打包工具的虚拟环境，并确认两个命令都在 `PATH` 中。 |
| `SDK needs libyaml-cpp.so... but it was not found` | 激活编译 SDK 时使用的 Conda 环境，或用 `YAML_CPP_SO=/path/to/libyaml-cpp.so ./python/package_wheel.sh 1.0.0 build/` 指定同一份动态库。 |
| `pip` 找不到 `.whl` 文件 | 先看打包是否成功，再从 SDK 根目录检查 `whl_build/wheelhouse/`。不要用上次残留的旧 wheel。 |
| 导入后只有 `RetCode.TIMEOUT` 等旧错误码 | 用上面的导入命令检查实际加载位置，确认新版已经重编译或重新安装。 |
