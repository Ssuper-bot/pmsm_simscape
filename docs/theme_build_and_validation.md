# 构建与验证流程

本文只讨论构建与验证路径，不解释控制理论。

## C++ 构建

构建配置来自 `cpp/CMakeLists.txt`。

主要选项：

- `BUILD_TESTS`（默认 ON）
- `BUILD_PYTHON_BINDINGS`（默认 OFF）

示例流程：

```bash
cd cpp
mkdir -p build && cd build
cmake .. -DBUILD_TESTS=ON
cmake --build .
```

默认核心静态库：`pmsm_core`

## C++ 测试

测试目标：`test_foc`

```bash
cd cpp/build
./test_foc
```

当前测试覆盖（见 `cpp/tests/test_foc_controller.cpp`）：

- Clarke / Park / 逆 Park
- SVPWM
- PI 控制器步进
- FOC 无状态接口
- FOC 有状态类接口

## Python 绑定构建

启用方式：

```bash
cd cpp
mkdir -p build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
cmake --build .
```

说明：

- CMake 会 `find_package(pybind11 QUIET)`。
- 找到后构建 `_pmsm_cpp` 模块，未找到仅给出 warning。

## MATLAB 侧构建与运行

初始化：

- `setup_project.m`

S-Function 编译：

- `matlab/scripts/build_sfun_foc.m`
- 编译源包括：`sfun_foc_controller.cpp` + C++ 核心源文件
- 编译选项包含 C++17 与 `-DMATLAB_MEX_FILE`

仿真入口：

- 独立仿真：`matlab/scripts/run_pmsm_simulation.m`
- Simscape 路径：`matlab/simscape/pmsm_foc_simscape.m`

模块化 Simulink 构建入口：

- `matlab/simscape/create_signal_in_module.m`
- `matlab/simscape/create_foc_controller_module.m`
- `matlab/simscape/create_three_invertor_module.m`
- `matlab/simscape/create_motor_module.m`
- `matlab/simscape/create_measure_module.m`
- `matlab/simscape/create_scope_module.m`
- `matlab/simscape/create_pmsm_foc_all_in_model.m`

模块化验证入口：

- `matlab/simscape/validate_pmsm_foc_modules.m`
- 验证内容依次包括：模型保存、`update diagram` 编译、短时仿真。
- 当前已验证通过：6 个独立模块 + `pmsm_foc_model` all-in 模型。
- 当前残留风险：all-in 模型存在 1 个代数环告警，但不阻塞短时仿真。
- 接口更新：`signal_in` 新增 `throttle` 生成链路并输出 `torque_ref`，all-in 模型连到 `FOC Controller` 第 8 输入。

## ROS2 运行入口

- Launch 文件：`python/launch/pmsm_system.launch.py`
- 启动节点：`pmsm-foc-node`、`pmsm-matlab-bridge`

## 验证顺序建议

1. 先验证 C++ 单元测试。
2. 再验证 MATLAB 独立仿真曲线。
3. 然后验证 Simscape 与 S-Function 路径。
4. 最后验证 ROS2 与 MATLAB 桥接端到端链路。

若要按模块迭代 Simulink：

1. 先运行对应 `create_*_module` 脚本，确认单模块结构和接口。
2. 再运行 `validate_pmsm_foc_modules(false)` 只检查模块。
3. 模块稳定后运行 `create_pmsm_foc_all_in_model` 或 `validate_pmsm_foc_modules(true)` 验证总装模型。

## 未验证/待确认

- pybind11、MATLAB Engine API、ROS2 工作区实际环境依赖未在本次确认。

## 关联文档

- 控制算法细节：./theme_control_core.md
- 仿真模型接口：./theme_simscape_simulation.md
- 目录与入口定位：./theme_repository_map.md
