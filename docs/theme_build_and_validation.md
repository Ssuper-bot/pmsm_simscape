# 构建与验证流程

本文只描述当前仓库里的构建、测试和验证入口，不展开控制理论。

## C++ 构建

构建脚本：`cpp/CMakeLists.txt`

主要选项：

- `BUILD_TESTS`：默认 `ON`
- `BUILD_PYTHON_BINDINGS`：默认 `OFF`

标准流程：

```bash
cd cpp
mkdir -p build
cd build
cmake .. -DBUILD_TESTS=ON
cmake --build .
```

当前默认构建目标：

- 静态库：`pmsm_core`
- 测试程序：`test_foc`（当 `BUILD_TESTS=ON`）
- pybind11 模块：`_pmsm_cpp`（当 `BUILD_PYTHON_BINDINGS=ON` 且 `pybind11` 可用）

## C++ 测试

执行方式：

```bash
cd cpp/build
ctest --output-on-failure
```

或直接执行：

```bash
./test_foc
```

当前覆盖范围来自 `cpp/tests/test_foc_controller.cpp`：

- Clarke / Park / inverse Park
- SVPWM
- PI 控制器步进行为
- FOC 无状态接口
- FOC 有状态类接口

## Python 绑定构建

启用方式：

```bash
cd cpp
mkdir -p build
cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
cmake --build .
```

行为说明：

- CMake 使用 `find_package(pybind11 QUIET)`。
- 找到后构建 `_pmsm_cpp`。
- 没找到时只发出 warning，不阻塞 C++ 核心库构建。

## MATLAB 侧入口

### 初始化

- `setup_project.m`

作用：

- 加入 `matlab/`、`matlab/scripts/`、`matlab/simscape/`、`matlab/s_function/`、`matlab/tests/` 到 MATLAB 路径。
- 打印当前可用命令入口。
- 支持 `setup_project('compare')`，初始化完成后直接打开 `compare_first_order_bandwidths` UI。

推荐用法：

```matlab
setup_project
setup_project('compare')
```

其中 `compare_first_order_bandwidths` 适合在正式跑 Simulink / Simscape 之前，先检查 PI 零极点相消、失配和纯 P 控制的闭环差异。

### 独立仿真

- `matlab/scripts/run_pmsm_simulation.m`

特点：

- 不依赖 Simulink / Simscape。
- 使用前向欧拉积分跑 dq 模型闭环。
- 启动时会自动编译并调用 `foc_controller_mex`。
- 适合先验证与 Simulink 同源的控制算法和默认参数。

### 控制器带宽对比

- `matlab/scripts/compare_first_order_bandwidths.m`

特点：

- 不依赖 Simulink / Simscape。
- 提供交互式 UI，对比 exact-cancel PI、mismatched PI 和 P-only 三种闭环。
- 可用于快速观察带宽、零点失配、稳态误差和频域响应差异。

### S-Function 编译

- `matlab/scripts/build_sfun_foc.m`

编译输入包括：

- `matlab/s_function/sfun_foc_controller.cpp`
- `cpp/src/foc_controller.cpp`
- `cpp/src/transforms.cpp`
- `cpp/src/pid_controller.cpp`
- `cpp/src/svpwm.cpp`

编译选项包含：

- C++17
- `-DMATLAB_MEX_FILE`
- MATLAB 现代 MEX API 选项 `-R2018a`

### 独立 MATLAB 控制器 MEX 编译

- `matlab/scripts/build_foc_mex.m`

编译输入包括：

- `cpp/mex/foc_controller_mex.cpp`
- `cpp/src/foc_controller.cpp`
- `cpp/src/transforms.cpp`
- `cpp/src/pid_controller.cpp`
- `cpp/src/svpwm.cpp`

用途：

- 让 `run_pmsm_simulation.m` 和 Simulink S-Function 复用同一套 C++ 控制实现。

### Simscape / Simulink 构建

主入口：

- `matlab/simscape/pmsm_foc_simscape.m`

该入口当前会：

1. 拉起默认参数。
2. 将参数注入 base workspace。
3. 强制编译 `sfun_foc_controller` MEX，因为 all-in 模型的 FOC 子系统已改为依赖该 C++ S-Function。
4. 删除旧的 `matlab/models/pmsm_foc_model.slx`。
5. 调用 `create_pmsm_foc_all_in_model` 重新生成 all-in 模型。

### 模块化 builder 入口

- `matlab/simscape/create_signal_in_module.m`
- `matlab/simscape/create_thro_module.m`
- `matlab/simscape/create_foc_controller_module.m`
- `matlab/simscape/create_three_invertor_module.m`
- `matlab/simscape/create_motor_module.m`
- `matlab/simscape/create_measure_module.m`
- `matlab/simscape/create_scope_module.m`
- `matlab/simscape/create_pmsm_foc_all_in_model.m`
- `matlab/simscape/create_pmsm_foc_model.m`（兼容性包装器）
- `matlab/simscape/pmsm_foc_builder.m`（共享底层逻辑）

## MATLAB 验证入口

### 模块和 all-in 验证

- `matlab/simscape/validate_pmsm_foc_modules.m`

当前行为：

- 依次生成 7 个模块 harness。
- 对每个模块执行保存、`update diagram`、短时仿真。
- `motor` 模块会额外断言包含 Simscape Electrical PMSM 块。
- 可选在最后验证 all-in `pmsm_foc_model`。

### 电机模块专项测试

- `matlab/tests/test_simscape_motor_module.m`

当前检查内容：

- standalone `motor` 模块中必须出现 Simscape PMSM 块。
- 必须出现三相电流传感器、转速传感器、转矩传感器。
- short smoke sim 中 `omega_m` 必须实际发生变化。
- all-in 模型中 `Motor` 子系统必须包含 Simscape PMSM。

## ROS2 运行入口

Launch 文件：`python/launch/pmsm_system.launch.py`

当前启动节点：

- `pmsm-foc-node`
- `pmsm-matlab-bridge`

默认 bridge 参数：

- `bridge_mode=file`
- `data_dir=/tmp/pmsm_bridge`

## 建议的验证顺序

1. 先跑 C++ 单元测试，确认算法基础行为没有退化。
2. 再跑 `compare_first_order_bandwidths`，先确认目标带宽和 PI 零点配置是否符合预期。
3. 然后跑 `run_pmsm_simulation`，确认闭环曲线大体合理。
4. 再跑 `pmsm_foc_simscape` 与 `validate_pmsm_foc_modules`，确认 builder、模块接口和短仿真都可用。
5. 需要电机结构回归时，再跑 `test_simscape_motor_module`。
6. 最后再做 ROS2 与 MATLAB 桥接联调。

## 当前已知差异与风险

- 独立 MATLAB 仿真默认 `Vdc = 24`，builder 默认 `Vdc = 48`。
- all-in 闭环仍可能出现代数环告警，这在文档层面应视为已知数值风险，不应误写成完全消除。
- Python/ROS2 环境、MATLAB Engine、pybind11 是否可用，仍依赖本机环境。

## 关联文档

- 控制算法细节：./theme_control_core.md
- 仿真模型接口：./theme_simscape_simulation.md
- 目录与入口定位：./theme_repository_map.md
