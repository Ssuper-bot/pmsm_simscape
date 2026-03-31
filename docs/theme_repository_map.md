# 仓库结构速览

本文只负责回答“文件在哪里、各目录负责什么”。

## 顶层目录职责

- `matlab/`：仿真模型、脚本、S-Function 与 MATLAB 函数库
- `cpp/`：FOC 控制核心、MEX/pybind 接口、CMake、C++ 测试
- `python/`：ROS2 节点、Python 绑定封装、launch 配置
- `config/`：预留配置目录
- `docs/`：项目文档

## 重点子目录

- `matlab/+pmsm_lib/`：变换、SVPWM、电机 dq 方程函数
- `matlab/simscape/`：Simscape 组件与模型生成脚本
- `matlab/models/`：Simulink 模型文件
- `cpp/include/` 与 `cpp/src/`：控制算法头文件与实现
- `cpp/tests/`：C++ 单元测试
- `python/pmsm_ros2/`：ROS2 逻辑节点与桥接实现

## 常见入口文件

- MATLAB 初始化：`setup_project.m`
- Simscape 主入口：`matlab/simscape/pmsm_foc_simscape.m`
- C++ 构建定义：`cpp/CMakeLists.txt`
- ROS2 启动入口：`python/launch/pmsm_system.launch.py`

## 与其他主题的关系

- 系统视角总览见 ./overview.md
- 算法细节见 ./theme_control_core.md
- 仿真体系见 ./theme_simscape_simulation.md
- 构建与验证见 ./theme_build_and_validation.md
