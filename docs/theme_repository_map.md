# 仓库结构速览

本文用于快速定位“哪类问题应该看哪个目录/文件”。

## 顶层目录职责

- `cpp/`：控制算法核心、测试、pybind/MEX 适配、CMake 构建
- `matlab/`：MATLAB 函数库、独立仿真脚本、Simscape 与 S-Function
- `python/`：ROS2 节点、绑定封装、launch 与 Python 包配置
- `docs/`：项目文档（当前主题化拆分）
- `config/`：配置预留目录（当前内容较少）

## C++ 相关地图

- 接口头文件：`cpp/include/`
- 核心实现：`cpp/src/`
- 单元测试：`cpp/tests/test_foc_controller.cpp`
- MEX 适配：`cpp/mex/foc_controller_mex.cpp`
- 构建脚本：`cpp/CMakeLists.txt`
- 当前存在构建目录：`cpp/build/`（含 `test_foc`）

## MATLAB 相关地图

- 算法函数库：`matlab/+pmsm_lib/`
- Simscape 脚本与组件：`matlab/simscape/`
- S-Function 源与产物：`matlab/s_function/`
- 运行脚本：`matlab/scripts/`
- Simulink 模型：`matlab/models/`
- MATLAB 测试：`matlab/tests/test_pmsm_lib.m`

## Python / ROS2 相关地图

- Python 包：`python/pmsm_ros2/`
- Launch：`python/launch/pmsm_system.launch.py`
- Python 工程配置：`python/pyproject.toml`
- ROS2 包清单：`python/package.xml`

## 常用入口文件

- MATLAB 初始化：`setup_project.m`
- Simscape 主入口：`matlab/simscape/pmsm_foc_simscape.m`
- Simulink 模块入口：`matlab/simscape/create_*_module.m`
- Simulink 总装入口：`matlab/simscape/create_pmsm_foc_all_in_model.m`
- Simulink 模块验证：`matlab/simscape/validate_pmsm_foc_modules.m`
- 独立仿真入口：`matlab/scripts/run_pmsm_simulation.m`
- C++ 构建入口：`cpp/CMakeLists.txt`
- ROS2 系统入口：`python/launch/pmsm_system.launch.py`

## 缓存/中间产物提示

- `slprj/`、`matlab/slprj/`：Simulink 生成缓存。
- `matlab/models/*.slxc`：模型缓存文件。
- 文档编写或检索问题时可优先忽略这些目录。

## 关联文档

- 全局导航：./overview.md
- 控制算法：./theme_control_core.md
- 仿真体系：./theme_simscape_simulation.md
- 构建验证：./theme_build_and_validation.md
- Python/ROS2：./theme_python_ros2_integration.md
