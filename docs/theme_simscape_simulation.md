# 仿真与模型体系

本文只讨论 MATLAB/Simulink/Simscape 仿真路径与接口，不展开 ROS2 系统编排。

## 关键文件

- 主配置脚本：`matlab/simscape/pmsm_foc_simscape.m`
- 模型创建脚本：`matlab/simscape/create_pmsm_foc_model.m`
- Simscape 电机组件：`matlab/simscape/pmsm_motor.ssc`
- S-Function 包装：`matlab/s_function/sfun_foc_controller.cpp`
- S-Function 编译脚本：`matlab/scripts/build_sfun_foc.m`
- 独立仿真脚本：`matlab/scripts/run_pmsm_simulation.m`
- 算法函数库：`matlab/+pmsm_lib/`

## 主流程 1：Simscape 模型路径

入口：`pmsm_foc_simscape.m`

行为要点：

- 检查 Simulink 可用性，不可用则提示改用 `run_pmsm_simulation`。
- 构建并注入 `motor_params`、`inv_params`、`ctrl_params`、`sim_params`、`ref_params` 到 base workspace。
- 若检测到 S-Function 源文件，调用 `build_sfun_foc` 编译 MEX。
- 删除旧 `pmsm_foc_model.slx` 后重新生成模型（保证脚本改动生效）。
- 当前生成逻辑已改为“模块先构建、最后总装”的结构：`pmsm_foc_simscape.m` 通过 `create_pmsm_foc_all_in_model.m` 调用共享 builder 生成 all-in 模型。

模块化入口：

- `create_signal_in_module.m`
- `create_foc_controller_module.m`
- `create_three_invertor_module.m`
- `create_motor_module.m`
- `create_measure_module.m`
- `create_scope_module.m`
- `create_pmsm_foc_all_in_model.m`
- `validate_pmsm_foc_modules.m`

模块职责：

- `signal_in`：速度参考、`id_ref`、负载阶跃输入、`throttle -> torque_ref`
- `foc_controller`：Clarke/Park/PI/InvPark/SVPWM 控制链；新增 `torque_ref` 在线换算 `iq_ref`
- `three-invertor`：三相平均逆变器
- `motor`：Simscape Electrical 内置 PMSM 电机子系统（含三相电气与机械端口）
- `measure`：电流透传、`theta_m -> theta_e`、速度输出
- `scope`：速度、电流、转矩观测

说明（2026-04 更新）：

- `motor` 模块已切换到 Simscape Electrical 内置 PMSM 块（`ee_lib/Electromechanical/Permanent Magnet/PMSM`），不再使用此前 `MATLAB Fcn + Integrator` 的自定义 dq 电机状态方程实现。
- 模块内部采用 Simscape 标准链路：三相受控电压源 + 三相电流传感器 + PMSM + 转矩/角速度传感器 + 负载转矩源。

总装模型：

- all-in 模型仍默认输出到 `matlab/models/pmsm_foc_model.slx`，因此旧入口和大部分既有脚本不需要改调用名。

验证方式：

- `validate_pmsm_foc_modules` 会依次为每个模块生成独立 harness 模型，执行保存、`update diagram`、短时仿真，并在最后验证 all-in 模型。

重要参数差异：

- 该路径默认 `inv_params.Vdc = 48`。

## 主流程 2：独立脚本仿真路径

入口：`run_pmsm_simulation.m`

行为要点：

- 不依赖 Simscape；按控制采样周期进行前向欧拉积分。
- 速度参考支持斜坡，负载支持阶跃扰动。
- 记录并绘制速度、电流、电压、占空比、电角度等曲线。

当前状态：

- `use_cpp_controller = false`（默认使用 MATLAB 实现）。
- C++ MEX 分支在脚本中保留占位，未直接启用（未验证/待确认）。

重要参数差异：

- 该路径默认 `inv_params.Vdc = 24`。

## S-Function 接口定义

文件：`matlab/s_function/sfun_foc_controller.cpp`

输入端口（1 个端口，宽度 7）：

1. `ia`
2. `ib`
3. `ic`
4. `theta_e`
5. `omega_m`
6. `speed_ref`
7. `id_ref`

模块化 builder（`pmsm_foc_builder.m`）当前 all-in 子系统接口额外包含：

- `FOC Controller` 第 8 输入：`torque_ref`（由 `Signal In` 的 `throttle` 线性映射后提供）
- `Signal In` 第 4 输出：`torque_ref`

`torque_ref -> iq_ref` 换算关系（在 FOC 子系统内）：

$$
K_t = 1.5\,p\,(\psi_f + (L_d-L_q)i_d),\quad
iq_{ref} = \begin{cases}
0,& |K_t| < 10^{-6}\\
\dfrac{T_e^{ref}}{K_t},& \text{otherwise}
\end{cases}
$$

输出端口（1 个端口，宽度 5）：

1. `da`
2. `db`
3. `dc`
4. `id_meas`
5. `iq_meas`

对话框参数（15 个）：

1. `Ts`
2. `Vdc`
3. `Kp_id`
4. `Ki_id`
5. `Kp_iq`
6. `Ki_iq`
7. `Kp_speed`
8. `Ki_speed`
9. `Rs`
10. `Ld`
11. `Lq`
12. `flux_pm`
13. `pole_pairs`
14. `iq_max`
15. `id_max`

状态管理：

- 使用 3 个 DWork 保存积分状态：`integral_id`、`integral_iq`、`integral_speed`。

## 待确认项

- Simscape 路径与独立脚本路径在默认 `Vdc` 参数上不一致，联调时需显式统一。
- all-in 模型当前短仿真可通过，但仍存在 1 个代数环告警，主要位于 `FOC Controller -> Three Invertor -> Motor -> Measure -> FOC Controller` 闭环；当前未阻塞构建验证，但后续若要做更稳定的数值仿真，建议引入离散控制采样或显式延时。

## Simscape PMSM 学习与使用

本仓库可直接参考的本地例程路径：

- `matlab/models/PMSMDrive_with_our_controller.slx`
- `matlab/models/pmsm_blue_plant_wrapper.slx`

推荐按以下顺序推进：

1. 学习模型：从 `PMSMDrive_with_our_controller.slx` 对照查看 PMSM 块参数字段（`nPolePairs`、`pm_flux_linkage`、`Ld`、`Lq`、`Rs`、`J`、`lam`）与电机侧传感器接线。
2. 使用模型：运行 `create_motor_module` 或 `create_pmsm_foc_all_in_model`，由 builder 自动生成采用 Simscape PMSM 的 motor 子系统。
3. 模块验证：运行 `validate_pmsm_foc_modules(false)`，确认每个模块保存、编译、短时仿真通过。
4. 总装验证：运行 `validate_pmsm_foc_modules(true)`，确认 all-in 模型中 `Motor` 子系统包含 Simscape PMSM 并可短时仿真。
5. 补充测试：运行 `matlab/tests/test_simscape_motor_module.m`，执行 motor 单模块和 all-in 的结构断言（含 PMSM 块存在性）与 smoke simulation。

## 关联文档

- 控制算法接口与流程：./theme_control_core.md
- 构建与验证命令：./theme_build_and_validation.md
- ROS2 桥接侧联调：./theme_python_ros2_integration.md
