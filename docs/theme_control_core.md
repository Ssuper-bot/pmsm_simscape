# 控制算法核心

本文聚焦 C++ 控制核心与接口定义，不展开 Simulink 画模或 ROS2 编排细节。

## 代码入口

- 主接口：`cpp/include/foc_controller.h`
- 主实现：`cpp/src/foc_controller.cpp`
- 坐标变换：`cpp/include/transforms.h`、`cpp/src/transforms.cpp`
- PI 控制器：`cpp/include/pid_controller.h`、`cpp/src/pid_controller.cpp`
- SVPWM：`cpp/include/svpwm.h`、`cpp/src/svpwm.cpp`

## 核心数据结构

### FOCConfig

用途：统一配置控制增益、电机参数、限幅、带宽和采样时间。

字段分组：

- 电流环：`Kp_id`、`Ki_id`、`Kp_iq`、`Ki_iq`
- 速度环：`Kp_speed`、`Ki_speed`
- 开关：`auto_tune_current`、`auto_tune_speed`、`enable_internal_speed_loop`
- 电机参数：`Rs`、`Ld`、`Lq`、`flux_pm`、`pole_pairs`、`J`、`B`
- 自动整定带宽：`omega_ci`、`omega_cs`
- 限幅：`iq_max`、`id_max`
- 系统参数：`Vdc`、`Ts`

### FOCOutput

用途：返回一次控制步的主要结果。

输出内容：

- PWM 占空比：`duty_a`、`duty_b`、`duty_c`
- dq 电流测量：`id_meas`、`iq_meas`
- dq 电压输出：`vd`、`vq`
- 电流环执行时实际使用的 `iq_ref`（已限幅）

## 自动整定逻辑

当前代码会通过 `derive_pi_gains_from_motor` 按开关进行 PI 自动整定：

- `auto_tune_current = true` 时，更新电流环 PI。
- `auto_tune_speed = true` 时，更新速度环 PI。
- 关闭自动整定时，保留用户手工设置的 PI 参数。

电流环设计：

$$
K_p = L\omega_{ci},\quad K_i = R_s\omega_{ci}
$$

速度环设计：

$$
K_p = \frac{J\omega_{cs}}{K_t},\quad K_i = \frac{B\omega_{cs}}{K_t}
$$

其中：

$$
K_t = 1.5\,p\,\psi_f
$$

默认带宽策略：

- 若 `omega_ci <= 0`，则按 $2\pi(f_{sw}/10)$ 推导。
- 若 `omega_cs <= 0`，则按 `omega_ci / 10` 推导。

## 对应的 MATLAB 对比工具

脚本：`matlab/scripts/compare_first_order_bandwidths.m`

用途：

- 用归一化一阶 RL 电流对象展示 PI 零极点相消的直观效果。
- 对比 exact-cancel PI、mismatched PI、P-only 三种闭环在阶跃、正弦、斜坡和频域下的差异。
- 帮助解释为什么当前自动整定在电流环里采用

$$
K_p = L\omega_{ci},\quad K_i = R_s\omega_{ci}
$$

因为此时有：

$$
\frac{K_i}{K_p} = \frac{R_s}{L} = \omega_e
$$

即控制器零点放在 $-\omega_e$，与一阶 RL 对象极点对齐。

## 控制接口分层

### 无状态速度环接口

函数：`speed_controller_step(...)`

主要用途：

- 速度环独立计算 `iq_ref`
- 需要外部维护 `integral_speed` 的场景（如 speed-core S-Function）

输入：`speed_ref`、`omega_m`、`integral_speed`、`FOCConfig`

### 无状态电流环接口

函数：`current_controller_step(...)`

主要用途：

- 电流环独立执行 dq PI、解耦前馈、限压和 SVPWM
- 需要外部维护 `integral_id`、`integral_iq` 的场景（如 current-core S-Function）

输入：`ia`、`ib`、`ic`、`theta_e`、`omega_m`、`id_ref`、`iq_ref`、积分状态、`FOCConfig`

### 无状态兼容接口

函数：`foc_controller_step(...)`

用途：兼容旧调用路径，内部根据 `enable_internal_speed_loop` 选择组合方式。

- 当 `enable_internal_speed_loop = true`：执行“速度环 + 转矩前馈 + 电流环”的组合流程。
- 当 `enable_internal_speed_loop = false`：把 `torque_ref` 入参按外部 `iq_ref` 解释，再直接走电流环。

### 有状态接口

类：`FOCController`

公开方法：

- `configure(const FOCConfig&)`
- `step(ia, ib, ic, theta_e, omega_m, speed_ref, id_ref, torque_ref=0)`
- `step_speed(speed_ref, omega_m)`
- `step_current(ia, ib, ic, theta_e, omega_m, id_ref, iq_ref)`
- `reset()`
- `config() const`

实现特征：

- 内部维护 `pi_id_`、`pi_iq_`、`pi_speed_` 三个 `PIController`。
- `configure()` 会先按开关执行自动整定，再下发 PI 与限幅参数。
- 速度环输出限幅为 `[-iq_max, iq_max]`。
- 电流环 PI 输出限幅为 `[-Vdc, Vdc]`，随后仍有电压矢量幅值限制。

## 算法主流程

### 推荐分环流程（当前 Simulink 主路径）

1. 速度环通过 `speed_controller_step` 产生 `iq_ref_speed`。
2. 外部前馈 `iq_ref_cmd` 与 `iq_ref_speed` 相加并限幅。
3. 电流环通过 `current_controller_step` 执行 `abc->dq`、PI、解耦、限压、SVPWM。

### 兼容流程（单入口）

1. `foc_controller_step` 判断 `enable_internal_speed_loop`。
2. 若开启内部速度环，则内部合成 `iq_ref`；若关闭，则使用外部 `iq_ref`。
3. 统一进入 `current_controller_step` 输出占空比与测量值。

## 变换与调制接口

- Clarke：`clarke(ia, ib, ic) -> AlphaBeta`
- Park：`park(alpha, beta, theta_e) -> DQ`
- inverse Park：`inv_park(d, q, theta_e) -> AlphaBeta`
- inverse Clarke：`inv_clarke(alpha, beta) -> ABC`
- SVPWM：`svpwm(v_alpha, v_beta, Vdc) -> DutyCycles`

## 当前限制与实现差异

- `current_controller_step` 里已对 `id_ref` 和 `iq_ref` 都做了对称限幅；但兼容入口 `foc_controller_step` 第 8 个入参仍命名为 `torque_ref`，在关闭内部速度环时语义实际是外部 `iq_ref`，需要调用方明确约定。
- 无状态接口对速度环有显式抗饱和回算；有状态接口则依赖 `PIController` 的内部限幅行为。
- 当前自动整定默认假设 $K_t = 1.5p\psi_f$，更复杂的磁阻转矩项并未进入默认 PI 推导。

## 关联文档

- 算法如何嵌入 Simulink：./theme_simscape_simulation.md
- Python / ROS2 如何调用：./theme_python_ros2_integration.md
- 构建与测试路径：./theme_build_and_validation.md
