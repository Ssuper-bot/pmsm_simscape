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
- 速度环输出：`iq_ref`

## 自动整定逻辑

当前代码会优先调用 `derive_pi_gains_from_motor`，按电机参数和目标带宽推导 PI 增益。

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

## 两类控制接口

### 无状态接口

函数：`foc_controller_step(...)`

主要用途：

- S-Function 集成
- 需要外部维护积分状态的调用场景

输入：

- 三相电流：`ia`、`ib`、`ic`
- 电角度与机械角速度：`theta_e`、`omega_m`
- 参考量：`speed_ref`、`id_ref`、`torque_ref`
- 外部维护的积分状态：`integral_id`、`integral_iq`、`integral_speed`
- 配置：`FOCConfig`

实现特征：

- 每步先对配置做自动整定。
- 速度环先输出 `iq_ref`，并带限幅与抗积分饱和回算。
- `torque_ref` 会按电机转矩常数在线换算为附加 `iq_ref`，再与速度环输出叠加后统一限幅。
- 电流环执行 dq PI 调节并叠加交叉耦合前馈。
- 电压矢量按 `Vdc / sqrt(3)` 做幅值限幅。
- 最后经 inverse Park 和 SVPWM 输出三相占空比。

### 有状态接口

类：`FOCController`

主要用途：

- Python 绑定
- 直接 C++ 调用

公开方法：

- `configure(const FOCConfig&)`
- `step(ia, ib, ic, theta_e, omega_m, speed_ref, id_ref, torque_ref=0)`
- `reset()`
- `config() const`

实现特征：

- 内部维护 `pi_id_`、`pi_iq_`、`pi_speed_` 三个 `PIController`。
- `configure()` 时会先自动整定，再把 PI 参数和输出限幅写入三个控制器。
- 速度环限幅是 `[-iq_max, iq_max]`。
- 电流环 PI 输出限幅是 `[-Vdc, Vdc]`，之后仍会再经过电压矢量幅值限制。

## 算法主流程

一次 `step` 的执行顺序可以概括为：

1. `abc -> alpha-beta` 的 Clarke 变换。
2. `alpha-beta -> dq` 的 Park 变换。
3. 速度环产生基础 `iq_ref`。
4. 将 `torque_ref` 在线换算为附加 `iq_ref` 并叠加后限幅。
5. d 轴和 q 轴 PI 产生 `vd`、`vq`。
6. 叠加解耦前馈项。
7. 进行电压矢量限幅。
8. inverse Park 回到 `alpha-beta`。
9. SVPWM 生成 `duty_a`、`duty_b`、`duty_c`。

## 变换与调制接口

- Clarke：`clarke(ia, ib, ic) -> AlphaBeta`
- Park：`park(alpha, beta, theta_e) -> DQ`
- inverse Park：`inv_park(d, q, theta_e) -> AlphaBeta`
- inverse Clarke：`inv_clarke(alpha, beta) -> ABC`
- SVPWM：`svpwm(v_alpha, v_beta, Vdc) -> DutyCycles`

## 当前限制与实现差异

- `id_max` 目前是配置字段，但在主控制流程中还没有看到与 `iq_max` 对称的显式 d 轴限幅逻辑。
- 无状态接口对速度环有显式抗饱和回算；有状态接口则依赖 `PIController` 的内部限幅行为。
- 当前自动整定默认假设 $K_t = 1.5p\psi_f$，更复杂的磁阻转矩项并未进入默认 PI 推导。

## 关联文档

- 算法如何嵌入 Simulink：./theme_simscape_simulation.md
- Python / ROS2 如何调用：./theme_python_ros2_integration.md
- 构建与测试路径：./theme_build_and_validation.md
