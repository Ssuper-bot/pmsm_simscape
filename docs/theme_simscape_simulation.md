# 仿真与模型体系

本文聚焦 MATLAB、Simulink、Simscape 相关路径与接口，不讨论 ROS2 编排细节。

## 关键文件

- 主入口：`matlab/simscape/pmsm_foc_simscape.m`
- 共享 builder：`matlab/simscape/pmsm_foc_builder.m`
- 总装入口：`matlab/simscape/create_pmsm_foc_all_in_model.m`
- 兼容入口：`matlab/simscape/create_pmsm_foc_model.m`
- Simscape 组件：`matlab/simscape/pmsm_motor.ssc`
- S-Function 包装：`matlab/s_function/sfun_foc_controller.cpp`
- S-Function 编译：`matlab/scripts/build_sfun_foc.m`
- 独立仿真：`matlab/scripts/run_pmsm_simulation.m`
- 模块验证：`matlab/simscape/validate_pmsm_foc_modules.m`
- 电机专项测试：`matlab/tests/test_simscape_motor_module.m`

## 主流程 1：模块化 Simulink / Simscape builder

入口：`pmsm_foc_simscape.m`

当前执行逻辑：

1. 检查 Simulink 是否可用。
2. 获取默认参数 `motor_params`、`inv_params`、`ctrl_params`、`sim_params`、`ref_params`。
3. 把参数注入 base workspace。
4. 如果检测到 `sfun_foc_controller.cpp`，尝试自动编译 MEX。
5. 删除旧 `matlab/models/pmsm_foc_model.slx`。
6. 调用 `create_pmsm_foc_all_in_model` 重新生成 all-in 模型。

### 模块入口

- `create_signal_in_module.m`
- `create_thro_module.m`
- `create_foc_controller_module.m`
- `create_three_invertor_module.m`
- `create_motor_module.m`
- `create_measure_module.m`
- `create_scope_module.m`

### 模块职责

- `signal_in`
	生成 `speed_ref`、`id_ref`、`load_torque`、`throttle`。
- `thro`
	根据当前速度、目标速度和油门开度生成 `torque_ref`。
- `foc_controller`
	现在是一个薄封装子系统：把测量量和参考量打包后送入 `sfun_foc_controller`，Clarke / Park / PI / inverse Park / SVPWM 与 `torque_ref -> iq_ref` 全部在 C++ 中实现。
- `three_invertor`
	生成三相平均电压输出。
- `motor`
	承载 Simscape PMSM 电机与机械链路。
- `measure`
	输出三相电流、机械角到电角的换算结果，以及转速量。
- `scope`
	聚合速度、电流、转矩等观测量。

## 当前电机实现

`motor` 模块当前主路径是 Simscape Electrical 内置 PMSM：

- PMSM 块：`ee_lib/Electromechanical/Permanent Magnet/PMSM`
- 三相电流传感器：`ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)`
- 机械转速传感器：`fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor`
- 转矩传感器：`fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor`

这意味着仓库当前默认建模路径已经不是“MATLAB Fcn + Integrator 的自定义 dq 电机”。

`pmsm_motor.ssc` 仍在仓库中，但不应在文档中被描述为当前 all-in 主模型的默认电机实现。

## all-in 模型

总装入口：`create_pmsm_foc_all_in_model.m`

默认连线关系：

- `Signal In -> FOC Controller`
- `Signal In -> Thro`
- `Measure -> Thro`
- `Thro -> FOC Controller`
- `FOC Controller -> Three Invertor`
- `Three Invertor -> Motor`
- `Motor -> Measure`
- `Measure -> FOC Controller`
- `Motor` 和 `FOC Controller` 部分输出进入 `Scope`

生成目标文件：

- `matlab/models/pmsm_foc_model.slx`

兼容性说明：

- `create_pmsm_foc_model.m` 现在只是调用 builder 的兼容包装器。

## 主流程 2：独立 MATLAB 脚本仿真

入口：`run_pmsm_simulation.m`

特点：

- 不依赖 Simulink 或 Simscape。
- 直接在 MATLAB 脚本里进行前向欧拉积分。
- 速度给定为斜坡，负载为阶跃。
- 记录速度、电流、电压、占空比、电角度等曲线。

当前脚本状态：

- 默认 `use_cpp_controller = false`。
- C++ MEX 控制器分支仍为预留占位，不应写成现成可直接启用的主路径。
- 默认 `inv_params.Vdc = 24`。

## Builder 默认参数特征

来自 `pmsm_foc_builder('default_parameters')`：

- `inv_params.Vdc = 48`
- `inv_params.fsw = 20e3`
- `ctrl_params.iq_max = 10`
- `ctrl_params.id_max = 10`
- `sim_params.validation_t_end = 0.02`
- `ref_params.throttle = 0.2`
- `ref_params.throttle_speed_kp = 0.05`
- `ref_params.throttle_torque_max = 0.8`

PI 参数会在 builder 内按电机参数自动推导。

## S-Function 接口定义

文件：`matlab/s_function/sfun_foc_controller.cpp`

S-Function 单端口输入宽度为 8：

1. `ia`
2. `ib`
3. `ic`
4. `theta_e`
5. `omega_m`
6. `speed_ref`（RPM）
7. `id_ref`
8. `torque_ref`

输出宽度为 5：

1. `da`
2. `db`
3. `dc`
4. `id_meas`
5. `iq_meas`

对话框参数共 13 个，覆盖采样时间、母线电压、带宽、电机参数和限幅。

状态管理方式：

- 3 个 DWork：`integral_id`、`integral_iq`、`integral_speed`

builder 当前不再在子系统层保留 MATLAB Fcn、PID block 或 `torque_ref -> iq_ref` 的本地实现；上述换算直接在 C++ 控制核心内完成。换算关系文档化如下：

$$
K_t = 1.5\,p\,(\psi_f + (L_d-L_q)i_d)
$$

$$
iq_{ref} =
\begin{cases}
0, & |K_t| < 10^{-6} \\
T_e^{ref} / K_t, & \text{otherwise}
\end{cases}
$$

## 验证方式

### 统一验证

- `validate_pmsm_foc_modules(false)`：只验证 7 个模块。
- `validate_pmsm_foc_modules(true)`：模块 + all-in 一起验证。

验证动作包括：

- 保存模型
- `update diagram`
- 短时仿真

### 电机专项验证

- `test_simscape_motor_module`

会额外断言：

- 电机模块确实包含 Simscape PMSM 与关键传感器。
- standalone motor 模块的 `omega_m` 在短时仿真中不是恒零。

## 已知差异与风险

- 独立脚本仿真和 builder 默认 `Vdc` 不一致。
- all-in 闭环仍可能存在代数环告警，当前不应写成“完全无告警”。
- 若要提高数值稳定性，后续仍可能需要进一步显式离散化控制链或增加延时环节。

## 可参考的模型

- `matlab/models/PMSMDrive_with_our_controller.slx`
- `matlab/models/pmsm_blue_plant_wrapper.slx`

这些文件更适合作为 Simscape PMSM 接线、参数字段和 plant 包装方式的对照样例。

## 关联文档

- 控制算法接口与流程：./theme_control_core.md
- 构建与验证命令：./theme_build_and_validation.md
- ROS2 桥接侧联调：./theme_python_ros2_integration.md
