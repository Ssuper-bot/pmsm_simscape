# 仿真与模型体系

本文只讨论 MATLAB / Simulink / Simscape 仿真模型与脚本，不展开 ROS2 系统集成。

## 主题范围

- Simscape 电机与系统模型创建
- Simulink 模型封装
- S-Function 接入 C++ 控制器
- MATLAB 侧独立仿真与验证脚本

## 关键文件位置

- Simscape 主脚本：`matlab/simscape/pmsm_foc_simscape.m`
- 模型创建脚本：`matlab/simscape/create_pmsm_foc_model.m`
- 电机组件：`matlab/simscape/pmsm_motor.ssc`
- 模型文件：`matlab/models/PMSMDrive_with_our_controller.slx`
- S-Function：`matlab/s_function/sfun_foc_controller.cpp`
- 工具脚本：`matlab/scripts/run_pmsm_simulation.m`
- MATLAB 函数库：`matlab/+pmsm_lib/*.m`

## 设计要点

- Simscape 路线用于物理一致性验证（电机、逆变器、负载与控制闭环）。
- S-Function 路线用于将 C++ 控制器嵌入 Simulink 以复用同一控制内核。
- 独立脚本路线用于快速算法验证，降低对完整工具链的依赖。

## 与其他主题的关系

- 控制算法细节见 ./theme_control_core.md
- 编译 MEX、运行测试见 ./theme_build_and_validation.md
- Python/ROS2 对接见 ./theme_python_ros2_integration.md
