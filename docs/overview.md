# 项目总览

本文是当前项目文档入口，已按仓库代码状态同步到 2026-04-14。

## 这次同步的主结论

- Simulink 主建模路径已经稳定为模块化 builder：先建 `signal_in`、`thro`、`foc_controller`、`three_invertor`、`motor`、`measure`、`scope`，再总装 all-in 模型。
- FOC 子系统当前为双 S-Function 架构：`Speed Core (sfun_speed_controller)` 负责速度环，`FOC Core (sfun_foc_controller)` 负责电流环与调制。
- `speed_ref` 仅进入速度环；current-core 输入宽度为 7（不含 `speed_ref`），对话框参数为 15 个（电流环必需字段 + 手动 PI + `auto_tune_current` 开关）。
- `motor` 模块当前主实现是 Simscape Electrical 内置 PMSM 块与配套传感器链路。
- C++ 控制核心提供无状态函数接口和有状态类接口，两者都使用基于电机参数的 PI 自动整定逻辑。
- `setup_project.m` 现在除了加路径，还会打印推荐入口，并支持 `setup_project('compare')` 直接打开一阶对象带宽对比 UI。
- Python / ROS2 层当前实际实现仍以 `std_msgs` 为主，代码注释中的自定义消息描述不应视为现状。

## 文档导航

- 控制算法核心：./theme_control_core.md
- 仿真与模型体系：./theme_simscape_simulation.md
- Python 与 ROS2 集成：./theme_python_ros2_integration.md
- 构建与验证流程：./theme_build_and_validation.md
- 仓库结构速览：./theme_repository_map.md
- C++ 源代码概览：./cpp_overview.md

## 推荐阅读路径

### 想看控制算法

1. `theme_control_core.md`
2. `theme_build_and_validation.md`
3. `theme_simscape_simulation.md`

### 想看 Simulink / Simscape

1. `theme_simscape_simulation.md`
2. `theme_repository_map.md`
3. `theme_build_and_validation.md`

### 想看 Python / ROS2 联调

1. `theme_python_ros2_integration.md`
2. `theme_build_and_validation.md`
3. `theme_control_core.md`

## 阅读前需要知道的差异

- 独立 MATLAB 脚本仿真默认 `Vdc = 24`。
- Simulink builder 默认 `Vdc = 48`。
- `compare_first_order_bandwidths` 是控制理论对比工具，不依赖 Simulink / Simscape，适合先看 PI 带宽和零点配置影响。
- `create_pmsm_foc_model.m` 现在主要是兼容性包装器，实际总装逻辑在 `pmsm_foc_builder.m` 与 `create_pmsm_foc_all_in_model.m`。

## 验证口径

- 本轮文档更新基于源码静态核对。
- 文档里提到的验证入口、测试文件和命令都来自仓库实际文件。
- 本轮未重新执行完整 MATLAB 仿真、ROS2 联调或全量构建，因此运行结果类结论仍应以实际环境复验为准。
