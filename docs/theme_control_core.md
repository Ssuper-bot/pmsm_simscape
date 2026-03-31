# 控制算法核心

本文只讨论控制算法与接口定义，不展开 ROS2 编排或 Simulink 画模细节。

## 代码入口

- 主接口：`cpp/include/foc_controller.h`
- 主实现：`cpp/src/foc_controller.cpp`
- 坐标变换：`cpp/include/transforms.h`、`cpp/src/transforms.cpp`
- PI 控制器：`cpp/include/pid_controller.h`、`cpp/src/pid_controller.cpp`
- SVPWM：`cpp/include/svpwm.h`、`cpp/src/svpwm.cpp`

## 核心数据结构

### FOCConfig

用途：统一配置电流环/速度环增益、电机参数、限幅与系统采样参数。

字段（来自 `FOCConfig`）：

- 电流环：`Kp_id`、`Ki_id`、`Kp_iq`、`Ki_iq`
- 速度环：`Kp_speed`、`Ki_speed`
- 电机参数：`Rs`、`Ld`、`Lq`、`flux_pm`、`pole_pairs`
- 限幅：`iq_max`、`id_max`
- 系统：`Vdc`、`Ts`

### FOCOutput

用途：返回控制器一步计算结果。

字段（来自 `FOCOutput`）：

- PWM 占空比：`duty_a`、`duty_b`、`duty_c`
- 电流观测：`id_meas`、`iq_meas`
- 电压输出：`vd`、`vq`
- 速度环输出：`iq_ref`

## 两类控制接口

### 无状态接口（推荐给 S-Function）

函数：`foc_controller_step(...)`

输入：

- 采样量：`ia`、`ib`、`ic`、`theta_e`、`omega_m`
- 参考量：`speed_ref`、`id_ref`
- 状态量（引用传入传出）：`integral_id`、`integral_iq`、`integral_speed`
- 配置：`config`

输出：`FOCOutput`

实现特征（`cpp/src/foc_controller.cpp`）：

- 速度环带 `iq_ref` 限幅与抗积分饱和回算。
- 电流环为 PI，含 dq 解耦前馈项。
- 电压矢量按 `Vdc/sqrt(3)` 幅值限幅。

### 有状态接口（推荐给 Python/C++ 直接调用）

类：`FOCController`

方法：

- `configure(const FOCConfig&)`
- `step(ia, ib, ic, theta_e, omega_m, speed_ref, id_ref)`
- `reset()`
- `config() const`

说明：内部维护 `pi_id_`、`pi_iq_`、`pi_speed_` 三个 `PIController` 状态。

## 变换与调制接口

- Clarke：`clarke(ia, ib, ic) -> AlphaBeta`
- Park：`park(alpha, beta, theta_e) -> DQ`
- 逆 Park：`inv_park(d, q, theta_e) -> AlphaBeta`
- 逆 Clarke：`inv_clarke(alpha, beta) -> ABC`
- SVPWM：`svpwm(v_alpha, v_beta, Vdc) -> DutyCycles`

## 限制与待确认

- `id_max` 当前只在配置结构中存在，主控制流程中未看到显式限幅逻辑（未验证/待确认是否为设计预期）。
- PI 抗饱和策略在无状态接口中有显式回算；有状态接口依赖 `PIController` 本身限幅行为。

## 关联文档

- 算法如何嵌入 Simulink：./theme_simscape_simulation.md
- Python/ROS2 如何调用：./theme_python_ros2_integration.md
- 构建与测试路径：./theme_build_and_validation.md
