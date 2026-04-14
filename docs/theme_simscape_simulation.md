# 仿真与模型体系

本文聚焦 MATLAB、Simulink、Simscape 相关路径与接口，不讨论 ROS2 编排细节。

## 关键文件

- 主入口：`matlab/simscape/pmsm_foc_simscape.m`
- 共享 builder：`matlab/simscape/pmsm_foc_builder.m`
- 总装入口：`matlab/simscape/create_pmsm_foc_all_in_model.m`
- 兼容入口：`matlab/simscape/create_pmsm_foc_model.m`
- Simscape 组件：`matlab/simscape/pmsm_motor.ssc`
- S-Function 包装（速度环）：`matlab/s_function/sfun_speed_controller.cpp`
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
4. 检测 speed/current 两个 S-Function 源并自动编译 MEX。
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
	当前实现为油门到 `iq_ref_ff` 的前馈通道：`throttle -> saturation -> gain -> iq saturation`。
	`current_speed` 与 `target_speed` 端口目前保留但不参与计算。
- `foc_controller`
	采用 `Speed Core + FOC Core` 双 S-Function：
	`Speed Core` 产生 `iq_ref_speed`，与外部 `iq_ref_cmd` 相加后限幅，再送入 `FOC Core`。
	`FOC Core` 只执行电流环与调制（不再接收 `speed_ref`）。
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

- 默认直接使用 `foc_controller_mex`，并在缺失时自动编译。
- MATLAB 脚本仿真与 Simulink S-Function 现在复用同一套 C++ 控制核心。
- 默认 `inv_params.Vdc = 24`。

## Builder 默认参数特征

来自 `pmsm_foc_builder('default_parameters')`：

- `inv_params.Vdc = 48`
- `inv_params.fsw = 20e3`
- `ctrl_params.iq_max = 10`
- `ctrl_params.id_max = 10`
- `ctrl_params.enable_internal_speed_loop = false`
- `sim_params.validation_t_end = 0.02`
- `ref_params.throttle = 0.0`
- `ref_params.throttle_iq_max = ctrl_params.iq_max`

PI 参数会在 builder 内按电机参数自动推导。

## S-Function 接口定义

### Speed Core

文件：`matlab/s_function/sfun_speed_controller.cpp`

- 输入宽度：2（`speed_ref` RPM，`omega_m` rad/s）
- 输出宽度：1（`iq_ref`）
- 对话框参数：4 个（`Ts`、`Kp_speed`、`Ki_speed`、`iq_max`）
- DWork：1 个（`integral_speed`）

### FOC Core（current-core）

文件：`matlab/s_function/sfun_foc_controller.cpp`

- 输入宽度：7
	1. `ia`
	2. `ib`
	3. `ic`
	4. `theta_e`
	5. `omega_m`
	6. `id_ref`
	7. `iq_ref`
- 输出宽度：5（`da`、`db`、`dc`、`id_meas`、`iq_meas`）
- 对话框参数：15 个（`Ts`、`Vdc`、`omega_ci`、`Rs`、`Ld`、`Lq`、`flux_pm`、`pole_pairs`、`iq_max`、`id_max`、`Kp_id`、`Ki_id`、`Kp_iq`、`Ki_iq`、`auto_tune_current`）
- DWork：2 个（`integral_id`、`integral_iq`）

### 子系统内合成关系

在 `foc_controller` 子系统内，`iq_ref` 的合成关系为：

$$
iq_{ref}^{core} = \mathrm{sat}\left(iq_{ref}^{speed} + iq_{ref}^{cmd}\right)
$$

其中 `iq_ref^speed` 来自 `Speed Core`，`iq_ref^cmd` 来自外部前馈（如 `thro` 模块输出）。

### 电流环 PI 调参语义

- `auto_tune_current = 1`：`FOC Core` 依据 `omega_ci` 与电机参数自动推导电流环 PI。
- `auto_tune_current = 0`：`FOC Core` 使用对话框中的 `Kp_id`、`Ki_id`、`Kp_iq`、`Ki_iq` 作为手动 PI 参数。

### 手动调参（在 MATLAB 中）

本节说明如何在 MATLAB base workspace 里临时覆盖控制器参数（`ctrl_params_override`），在不改源码的情况下手动设置电流环/速度环 PI 参数并重建仿真模型。

步骤：

1. （可选）预编译 S-Function（避免每次运行时重新编译）：
```matlab
% 只需执行一次
addpath(fullfile(pwd,'matlab','scripts'));
build_sfun_foc(fullfile(pwd,'matlab','s_function'));
```

2. 在 base workspace 设置 `ctrl_params_override`：
```matlab
ctrl_params_override = struct();
% 电流环（关闭自动整定）
ctrl_params_override.auto_tune_current = 0;
ctrl_params_override.Kp_id = 8;
ctrl_params_override.Ki_id = 3000;
ctrl_params_override.Kp_iq = 8;
ctrl_params_override.Ki_iq = 3000;
% 速度环（关闭自动整定）
ctrl_params_override.auto_tune_speed = 0;
ctrl_params_override.Kp_speed = 0.12;
ctrl_params_override.Ki_speed = 1.5;
% 可选：显式带宽/限幅
ctrl_params_override.omega_ci = 12566.37;
ctrl_params_override.iq_max = 10;
```

3. 生成/重建模型并把参数写回 workspace：
```matlab
pmsm_foc_simscape; % 会读取 base workspace 的 *_override 并生成模型
% 或者只生成参数并用 builder 手动生成模型（不强制编译 MEX）
[motor_params,inv_params,ctrl_params,sim_params,ref_params] = pmsm_foc_builder('default_parameters');
fn = fieldnames(ctrl_params_override); for i=1:numel(fn); ctrl_params.(fn{i}) = ctrl_params_override.(fn{i}); end
pmsm_foc_builder('create_all_in_model','pmsm_foc_model',motor_params,inv_params,ctrl_params,sim_params,ref_params);
```

4. 运行仿真并检查结果：
```matlab
sim('pmsm_foc_model','StopTime','0.08');
% 或用 builder 的验证接口
pmsm_foc_builder('validate_model','pmsm_foc_model',0.08);
```

5. 验证 S-Function 接收到的参数：
```matlab
get_param('pmsm_foc_model/FOC Controller/FOC Core','Parameters')
get_param('pmsm_foc_model/FOC Controller/Speed Core','Parameters')
% 查看合并后的 ctrl_params
disp(ctrl_params)
```

6. 清理 override，恢复默认行为：
```matlab
clear ctrl_params_override
pmsm_foc_simscape; % 或重新运行 builder
```

注意事项：
- `pmsm_foc_builder('derive_pi_ctrl_params',...)` 会在 `auto_tune_current==1` 或 `auto_tune_speed==1` 时自动覆盖 PI 参数，若要使用手动值，务必把对应的 `auto_tune_*` 设置为 `0`。
- `pmsm_foc_simscape` 当前会尝试（默认）编译 S-Function MEX；如不希望每次编译，先预编译或修改脚本（见本文件“快速工作流”）。
- 过高的带宽或 Kp/Ki 会导致占空比饱和与跟踪失稳；在调增带宽时同时监测占空比饱和命中数（`probe` 或 `Scope`）并逐步增益。

示例：典型起点
- `omega_ci ≈ 2π*(fsw/10)`（builder 默认）
- 若 `omega_ci=12566.37`，推导结果可能为 `Kp_id≈17.6, Ki_id≈6283`（可据此作为参考初值）。

## 验证方式

### 统一验证

- `validate_pmsm_foc_modules(false)`：只验证 7 个模块。
- `validate_pmsm_foc_modules(true)`：模块 + all-in 一起验证。

验证动作包括：

- 保存模型
- `update diagram`
- 短时仿真

## 当前断环策略

all-in 模型当前在 `Three Invertor` 子系统入口加入了一个控制周期的 `Unit Delay`。

目的：

- 表达数字控制链中占空比命令到功率级生效的一个采样周期延时。
- 打断 `Motor -> Measure -> FOC Controller -> Three Invertor -> Motor` 的直接代数环。

### 电机专项验证

- `test_simscape_motor_module`

会额外断言：

- 电机模块确实包含 Simscape PMSM 与关键传感器。
- standalone motor 模块的 `omega_m` 在短时仿真中不是恒零。

## 已知差异与风险

- 独立脚本仿真和 builder 默认 `Vdc` 不一致。
- 虽然当前代数环告警已经通过功率级前的一拍延时打断，但闭环动态仍受该离散延时影响，调参时需要按这个执行语义理解结果。
- 若要进一步提高数值稳定性，后续仍可能需要更细的采样/求解器联调。

## 可参考的模型

- `matlab/models/PMSMDrive_with_our_controller.slx`
- `matlab/models/pmsm_blue_plant_wrapper.slx`

这些文件更适合作为 Simscape PMSM 接线、参数字段和 plant 包装方式的对照样例。

## 关联文档

- 控制算法接口与流程：./theme_control_core.md
- 构建与验证命令：./theme_build_and_validation.md
- ROS2 桥接侧联调：./theme_python_ros2_integration.md
