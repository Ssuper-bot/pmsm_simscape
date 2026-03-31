# 项目总览

本文是项目文档入口，内容基于当前仓库代码状态（2026-03-31）同步。

## 项目大纲

1. 控制算法核心
2. 仿真与模型体系
3. Python 与 ROS2 集成
4. 构建与验证流程
5. 仓库结构速览

## 文档导航

- 控制算法核心：./theme_control_core.md
- 仿真与模型体系：./theme_simscape_simulation.md
- Python 与 ROS2 集成：./theme_python_ros2_integration.md
- 构建与验证流程：./theme_build_and_validation.md
- 仓库结构速览：./theme_repository_map.md

## 当前代码主干结论

- C++ 核心提供两种 FOC 接口：无状态函数接口与有状态类接口。
- MATLAB 路径同时支持独立脚本仿真与 Simscape/Simulink 模型流程。
- Python 层对 C++ 绑定提供导入失败时的纯 Python 回退实现。
- ROS2 目前使用 `std_msgs/Float64` 与 `std_msgs/Float64MultiArray` 进行消息交互。

## 验证状态说明

- 文档中的接口、参数、话题、命令来自源码静态核对。
- 本次未执行完整构建/仿真/ROS2 运行链路，运行结果相关内容统一视为“未验证/待确认”。

## 推荐阅读路径

- 算法实现优先：先看“控制算法核心”，再看“构建与验证流程”。
- 仿真建模优先：先看“仿真与模型体系”，再看“控制算法核心”。
- 系统联调优先：先看“Python 与 ROS2 集成”，再看“构建与验证流程”。
