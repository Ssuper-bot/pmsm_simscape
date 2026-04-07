# Python 与 ROS2 集成

本文描述 Python 包、ROS2 节点接口和 MATLAB 桥接行为，不讨论控制公式推导。

## 关键文件

- Python 包：`python/pmsm_ros2/`
- FOC 节点：`python/pmsm_ros2/foc_node.py`
- 绑定与回退层：`python/pmsm_ros2/foc_bindings.py`
- MATLAB 桥接节点：`python/pmsm_ros2/matlab_bridge.py`
- Launch：`python/launch/pmsm_system.launch.py`
- Python 打包配置：`python/pyproject.toml`
- ROS2 包元信息：`python/package.xml`

## 可执行入口

来自 `python/pyproject.toml`：

- `pmsm-foc-node` -> `pmsm_ros2.foc_node:main`
- `pmsm-matlab-bridge` -> `pmsm_ros2.matlab_bridge:main`

## FOC ROS2 节点

文件：`python/pmsm_ros2/foc_node.py`

### 订阅话题

- `/pmsm/feedback`：`std_msgs/Float64MultiArray`
  数据顺序：`[ia, ib, ic, theta_e, omega_m]`
- `/pmsm/speed_ref`：`std_msgs/Float64`

### 发布话题

- `/pmsm/pwm_duty`：`std_msgs/Float64MultiArray`
  数据：`[duty_a, duty_b, duty_c]`
- `/pmsm/status`：`std_msgs/Float64MultiArray`
  数据：`[id_meas, iq_meas, vd, vq, iq_ref, speed_ref, omega_m]`

### 节点参数

- 电机：`motor.Rs`、`motor.Ld`、`motor.Lq`、`motor.flux_pm`、`motor.pole_pairs`
- 控制器：`controller.Kp_id`、`controller.Ki_id`、`controller.Kp_iq`、`controller.Ki_iq`、`controller.Kp_speed`、`controller.Ki_speed`、`controller.iq_max`、`controller.id_max`
- 逆变器：`inverter.Vdc`、`inverter.fsw`

运行时会通过：

$$
T_s = 1 / f_{sw}
$$

生成控制器采样时间。

### 实现现状说明

- 文件顶部注释里仍保留 `pmsm_msgs/*` 的旧描述。
- 但当前真实代码实现已经统一为 `std_msgs/*`，文档应以源码行为为准。

## 绑定层行为

文件：`python/pmsm_ros2/foc_bindings.py`

当前策略：

1. 先尝试导入编译后的 `_pmsm_cpp`。
2. 若导入失败，回退到纯 Python 版 `FOCConfig`、`FOCOutput`、`FOCController` 和变换/SVPWM 实现。

这意味着：

- 开发环境即使没有 pybind11 产物，也能先把 ROS2 节点跑起来。
- 但数值行为、性能和限幅细节是否完全与 C++ 一致，仍应按实际场景再做比对。

## MATLAB 桥接节点

文件：`python/pmsm_ros2/matlab_bridge.py`

支持两种模式：

- `bridge_mode=file`：默认，通过 JSON 文件读写。
- `bridge_mode=engine`：通过 MATLAB Engine API 直接交互。

默认参数：

- `bridge_mode=file`
- `data_dir=/tmp/pmsm_bridge`
- `matlab_model=pmsm_foc_model`

### 文件模式下的数据交换

写给 MATLAB：

- `pwm_input.json`
- `status.json`

从 MATLAB 读取：

- `motor_feedback.json`

桥接节点会把 `motor_feedback.json` 重新发布到 `/pmsm/feedback`。

## Launch 编排

文件：`python/launch/pmsm_system.launch.py`

当前会启动：

- `pmsm-foc-node`
- `pmsm-matlab-bridge`

并为 FOC 节点注入默认电机、控制器、逆变器参数。

## 打包与环境注意事项

- `pyproject.toml` 是标准 Python 包入口配置。
- `package.xml` 当前同时声明了 `ament_cmake`、`ament_cmake_python`，并在导出段声明 `ament_python`。
- 这说明 ROS2 工作区的实际打包方式仍需以本地环境和 colcon 配置为准，文档不应把它写成已经完全收敛的发布流程。

## 当前风险与待确认

- `_pmsm_cpp` 是否可导入取决于本机 pybind11 构建产物。
- MATLAB Engine 模式依赖单独安装 `matlab.engine`。
- ROS2 打包链路的一致性还需要在真实工作区再验证一次。

## 关联文档

- 控制核心接口：./theme_control_core.md
- 仿真与 Simscape：./theme_simscape_simulation.md
- 构建与测试路径：./theme_build_and_validation.md
