# 仓库结构速览

本文用于快速回答两个问题：

1. 某个问题应该先看哪个目录。
2. 当前项目的真实入口文件是什么。

## 顶层目录职责

- `cpp/`：FOC 算法核心、CMake 构建、单元测试、pybind11 与 MEX 复用源。
- `matlab/`：MATLAB 函数库、独立仿真脚本、Simscape/Simulink builder、S-Function、MATLAB 测试。
- `python/`：ROS2 Python 包、launch、Python 打包元信息。
- `docs/`：当前主题化文档。
- `config/`：预留配置目录。
- `attachment/`：附件和实验结果归档目录。

## C++ 相关地图

- 接口头文件：`cpp/include/`
- 核心实现：`cpp/src/`
- 单元测试：`cpp/tests/test_foc_controller.cpp`
- MEX 适配：`cpp/mex/foc_controller_mex.cpp`
- 构建脚本：`cpp/CMakeLists.txt`
- 常见构建产物：`cpp/build/`

关注点建议：

- 控制律和自动整定：看 `foc_controller.*`
- 坐标变换：看 `transforms.*`
- PI 行为：看 `pid_controller.*`
- PWM 占空比输出：看 `svpwm.*`

## MATLAB 相关地图

- 算法函数库：`matlab/+pmsm_lib/`
- Simscape / Simulink builder：`matlab/simscape/`
- S-Function 源与 MEX 产物：`matlab/s_function/`
- 运行脚本：`matlab/scripts/`
- Simulink 模型与参考模型：`matlab/models/`
- MATLAB 测试：`matlab/tests/`

`matlab/simscape/` 当前最关键的入口：

- `pmsm_foc_builder.m`：共享 builder，负责参数默认值、模块创建、all-in 创建、模型验证。
- `pmsm_foc_simscape.m`：主入口，加载参数、尝试构建 S-Function、重新生成 all-in 模型。
- `create_*_module.m`：单模块入口。
- `create_pmsm_foc_all_in_model.m`：总装入口。
- `create_pmsm_foc_model.m`：兼容旧调用名的包装器。
- `validate_pmsm_foc_modules.m`：模块与 all-in 的统一验证入口。

## Python / ROS2 相关地图

- Python 包：`python/pmsm_ros2/`
- Launch：`python/launch/pmsm_system.launch.py`
- Python 工程配置：`python/pyproject.toml`
- ROS2 包清单：`python/package.xml`

Python 包内部职责：

- `foc_node.py`：ROS2 控制节点，订阅反馈与速度给定，发布 PWM 和状态。
- `foc_bindings.py`：优先导入 `_pmsm_cpp`，失败时切回纯 Python 实现。
- `matlab_bridge.py`：文件模式或 MATLAB Engine 模式桥接 MATLAB/Simulink。

## 常用入口文件

- MATLAB 初始化：`setup_project.m`
- 独立仿真：`matlab/scripts/run_pmsm_simulation.m`
- 独立仿真 MEX 编译：`matlab/scripts/build_foc_mex.m`
- S-Function 编译：`matlab/scripts/build_sfun_foc.m`
- Simscape 主入口：`matlab/simscape/pmsm_foc_simscape.m`
- 模块验证：`matlab/simscape/validate_pmsm_foc_modules.m`
- 电机模块专项测试：`matlab/tests/test_simscape_motor_module.m`
- C++ 构建入口：`cpp/CMakeLists.txt`
- ROS2 系统入口：`python/launch/pmsm_system.launch.py`

## 参考模型与辅助文件

- `matlab/models/PMSMDrive_with_our_controller.slx`：本地 Simscape PMSM 参考模型。
- `matlab/models/pmsm_blue_plant_wrapper.slx`：blue plant 包装模型示例。
- `matlab/models/pmsm_foc_model.slx`：builder 生成的 all-in 模型目标文件。

## 可以优先忽略的目录

- `slprj/`、`matlab/slprj/`：Simulink 缓存。
- `*.slxc`、`*.autosave`：模型缓存或自动保存文件。
- `cpp/build/`：构建中间产物和可执行文件。

## 关联文档

- 全局导航：./overview.md
- 控制算法：./theme_control_core.md
- 仿真体系：./theme_simscape_simulation.md
- 构建验证：./theme_build_and_validation.md
- Python / ROS2：./theme_python_ros2_integration.md
