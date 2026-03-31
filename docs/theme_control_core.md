# 控制算法核心

本文只讨论控制算法本体，不展开构建与部署细节。

## 主题范围

- FOC 主控制流程
- Clarke / Park / 逆 Park 变换
- PI 电流环控制（含积分状态）
- SVPWM 调制

## 关键代码位置

- C++ 接口：`cpp/include/foc_controller.h`
- FOC 实现：`cpp/src/foc_controller.cpp`
- 变换模块：`cpp/include/transforms.h`、`cpp/src/transforms.cpp`
- PI 模块：`cpp/include/pid_controller.h`、`cpp/src/pid_controller.cpp`
- SVPWM 模块：`cpp/include/svpwm.h`、`cpp/src/svpwm.cpp`

## 核心信号链（概念）

1. 采样三相电流与电角度
2. abc -> alpha-beta（Clarke）
3. alpha-beta -> dq（Park）
4. d/q 电流误差进入 PI 控制器，得到 v_d / v_q
5. dq -> alpha-beta（逆 Park）
6. SVPWM 计算占空比并输出给逆变器

## 与其他主题的关系

- 若关注模型中如何嵌入该算法：见 ./theme_simscape_simulation.md
- 若关注 Python 调用和 ROS2 节点：见 ./theme_python_ros2_integration.md
- 若关注如何编译与测试该算法：见 ./theme_build_and_validation.md
