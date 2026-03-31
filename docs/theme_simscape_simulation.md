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

- `create_pmsm_foc_model.m` 具体连线与模块细节本文未展开（未验证/待确认）。
- Simscape 路径与独立脚本路径在默认 `Vdc` 参数上不一致，联调时需显式统一。

## 关联文档

- 控制算法接口与流程：./theme_control_core.md
- 构建与验证命令：./theme_build_and_validation.md
- ROS2 桥接侧联调：./theme_python_ros2_integration.md
