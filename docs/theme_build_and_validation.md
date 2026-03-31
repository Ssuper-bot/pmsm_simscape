# 构建与验证流程

本文只讨论如何构建与验证，不解释算法公式或模型原理。

## 主题范围

- C++ 目标构建
- C++ 单元测试
- MATLAB MEX / S-Function 构建
- Python 绑定构建
- 常用验证入口

## C++ 构建与测试

1. 进入 `cpp/build`
2. 配置 CMake（可选开启测试）
3. 编译目标
4. 运行测试二进制（例如 `test_foc`）

## MATLAB 相关构建

- 项目初始化脚本：`setup_project.m`
- S-Function 构建脚本：`matlab/scripts/build_sfun_foc.m`
- 独立仿真脚本：`matlab/scripts/run_pmsm_simulation.m`

## Python/绑定相关

- 绑定实现：`cpp/src/pybind_module.cpp`
- Python 包入口：`python/pmsm_ros2/`
- 启动脚本：`python/launch/pmsm_system.launch.py`

## 验证建议

- 先跑 C++ 单元测试，确认控制核心正确性。
- 再跑 MATLAB 独立仿真，检查控制闭环趋势。
- 最后再跑 ROS2 集成链路，验证消息与系统联动。

## 与其他主题的关系

- 算法说明见 ./theme_control_core.md
- 仿真模型说明见 ./theme_simscape_simulation.md
- 仓库定位见 ./theme_repository_map.md
