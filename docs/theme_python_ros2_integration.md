# Python 与 ROS2 集成

本文只讨论 Python 包、节点接口与桥接行为，不展开控制公式推导。

## 关键文件

- 包目录：`python/pmsm_ros2/`
- FOC ROS2 节点：`python/pmsm_ros2/foc_node.py`
- 绑定与回退层：`python/pmsm_ros2/foc_bindings.py`
- MATLAB 桥接节点：`python/pmsm_ros2/matlab_bridge.py`
- Launch：`python/launch/pmsm_system.launch.py`
- Python 入口配置：`python/pyproject.toml`
- ROS2 包元信息：`python/package.xml`

## 可执行入口

来自 `python/pyproject.toml` 的脚本入口：

- `pmsm-foc-node` -> `pmsm_ros2.foc_node:main`
- `pmsm-matlab-bridge` -> `pmsm_ros2.matlab_bridge:main`

## FOC 节点接口

文件：`python/pmsm_ros2/foc_node.py`

### 订阅话题

- `/pmsm/feedback`：`Float64MultiArray`
	预期数据顺序：`[ia, ib, ic, theta_e, omega_m]`
- `/pmsm/speed_ref`：`Float64`

### 发布话题

- `/pmsm/pwm_duty`：`Float64MultiArray`
	数据：`[duty_a, duty_b, duty_c]`
- `/pmsm/status`：`Float64MultiArray`
	数据：`[id_meas, iq_meas, vd, vq, iq_ref, speed_ref, omega_m]`

### 关键参数

- 电机：`motor.Rs`、`motor.Ld`、`motor.Lq`、`motor.flux_pm`、`motor.pole_pairs`
- 控制器：`controller.Kp_id`、`controller.Ki_id`、`controller.Kp_iq`、`controller.Ki_iq`、`controller.Kp_speed`、`controller.Ki_speed`、`controller.iq_max`、`controller.id_max`
- 逆变器：`inverter.Vdc`、`inverter.fsw`

其中采样时间由 `Ts = 1.0 / inverter.fsw` 计算。

## 绑定层行为

文件：`python/pmsm_ros2/foc_bindings.py`

- 优先导入 C++ 模块 `_pmsm_cpp`。
- 导入失败时使用纯 Python 回退实现（包含 `FOCConfig`、`FOCOutput`、`FOCController` 与变换/SVPWM函数）。

意义：

- 开发调试场景下不阻塞 Python 层运行。
- 但性能与数值行为需与 C++ 实现额外比对（未验证/待确认）。

## MATLAB 桥接节点

文件：`python/pmsm_ros2/matlab_bridge.py`

支持两种模式：

- `bridge_mode=file`（默认）：通过 JSON 文件交换数据。
- `bridge_mode=engine`：通过 MATLAB Engine API 交互。

默认目录参数：`data_dir=/tmp/pmsm_bridge`

文件模式下的典型数据文件：

- 写入 MATLAB：`pwm_input.json`、`status.json`
- 从 MATLAB 读取：`motor_feedback.json`

## Launch 编排

文件：`python/launch/pmsm_system.launch.py`

- 启动 `pmsm-foc-node`，并注入电机/控制器/逆变器参数。
- 启动 `pmsm-matlab-bridge`，默认 `bridge_mode=file`。

## 待确认项

- `package.xml` 中同时出现 `ament_cmake` 与 `ament_python` 相关声明，当前打包流程一致性未验证。
- 文档字符串提到自定义消息类型（`pmsm_msgs/*`），但当前实际代码使用 `std_msgs/*`（以源码实现为准）。

## 关联文档

- 控制核心接口：./theme_control_core.md
- 仿真与 Simscape：./theme_simscape_simulation.md
- 构建与测试路径：./theme_build_and_validation.md
