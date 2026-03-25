# PMSM Simscape Session Handoff Document / 项目会话交接文档（2026-03-24）

## 1. 本次会话目标与用户确认

### 用户最初目标
- 三层架构方向：C++ 做算法，MATLAB 做仿真，Python/ROS2 做后续集成。
- 当前仓库“有架构但离目标较远”，希望先明确需求并形成可执行路径。

### 已完成需求澄清（用户已确认）
- 电机类型：先做 SPMSM。
- 第一阶段可暂缓 Python/ROS2 闭环。
- 仿真目标：优先“Simscape 逆变器开关细节级别”场景。
- 传感器：需要考虑噪声，不是假设理想测量。
- 首要问题：在 Simulink 可运行，但接入 Simscape 后无法正常运行。
- 第一阶段工况：速度阶跃 + 负载阶跃。
- 第一阶段验收：只要 C++ 控制器在 Simscape 中跑通即可，不设性能指标。
- 用户确认使用默认测试值并直接开工。

## 2. 会话中形成的关键判断

- 现有仓库在控制和仿真层更偏“架构样板”，并非真正打通的闭环验证系统。
- 现有所谓 Simscape 主流程里，模型生成脚本核心是平均值/方程型 Simulink 子系统，而非明确构建的开关器件级物理网络。
- 存在“以为使用 C++ 控制器、实际可能回退 MATLAB 实现”的风险。
- 因此第一阶段策略是：
  - 收缩范围到 MATLAB/Simscape + C++ S-Function；
  - 强制 C++ 控制器路径；
  - 不允许静默回退；
  - 先追求可跑通，再谈性能。

## 3. 本次已完成代码改动

### 3.1 模型生成器改造
文件：matlab/simscape/create_pmsm_foc_model.m

已做内容：
- 增加控制器模式分支，支持 controller_mode = sfun。
- 新增 S-Function 控制器子系统创建函数。
- 在 sfun 模式下，FOC 子系统使用 sfun_foc_controller 作为核心控制块。
- 统一映射 S-Function 输入输出：
  - 输入 7 路：ia ib ic theta_e omega_m speed_ref id_ref
  - 输出 5 路：da db dc id_meas iq_meas
- 测量子系统增加噪声注入：
  - ia ib ic 电流测量噪声
  - theta_e 角度噪声
- 噪声参数可通过 sim_params 配置：
  - current_noise_std
  - theta_noise_std
  - noise_seed

### 3.2 Simscape 主入口改造
文件：matlab/simscape/pmsm_foc_simscape.m

已做内容：
- 脚本改为函数形式：pmsm_foc_simscape(overrides)。
- 默认第一阶段参数已固化：
  - speed_ref = 1000 rpm
  - load_torque = 0.1 N*m
  - load_step_time = 0.3 s
  - t_end = 0.5 s
  - controller_mode = sfun
  - current_noise_std = 0.02 A
  - theta_noise_std = 1e-3 rad
- 增加参数覆盖机制（overrides）。
- 构建 S-Function 时改为“失败即报错”，不再静默回退。
- 构建后检查 sfun_foc_controller 是否可见，否则报错。
- 默认可直接运行 sim（run_sim=true）。

### 3.3 第一阶段验收脚本
文件：matlab/scripts/run_phase1_acceptance.m

已做内容：
- 新增两工况自动运行：
  - speed_step_only
  - speed_and_load_step
- 当前验收判据：仿真不报错、可完整运行。
- 输出每个 case 的 PASS/FAIL 以及错误消息。

### 3.4 本轮追加修复（2026-03-24，当天续修）
文件：matlab/simscape/create_pmsm_foc_model.m

背景：
- 用户在 MATLAB 执行 pmsm_foc_simscape 后，先报语法错误：
  - create_pmsm_foc_model.m 行 519 列 47
  - 无效表达式（提示括号或分隔符不匹配）
- 语法修复后，进入仿真阶段又出现维度错误：
  - pmsm_foc_model/Power Stage/Demux 输入维度无效
  - pmsm_foc_model/Power Stage/Modulator 输出被识别为 1 元素，和三相链路不匹配

本轮已完成改动：
1) 修复 Measurement 子系统损坏代码段（语法层）
- 清理重复/断裂的 Random Number 与 add_line 片段。
- 恢复完整端口定义：
  - 输入 5 路：ia ib ic theta_m omega_m_in
  - 输出 5 路：ia_meas ib_meas ic_meas theta_e omega_m
- 保持噪声注入方向不变：
  - ia ib ic 各自白噪声叠加
  - theta_e 由 pole_pairs * theta_m 再叠加角度噪声
  - omega_m 直通输出

2) 修复 S-Function 占空比输出接线（维度层）
- 现有 sfun_foc_controller 输出为 5 路：da db dc id_meas iq_meas。
- 之前误把 Demux_out/1 直接接到 duty_abc，导致占空比变成标量。
- 现改为把 Demux_out/1..3 先进入 Mux_duty，再输出到 duty_abc，恢复三相向量。

本轮调试过程（已实机执行）：
- 使用 MATLAB 可执行文件：/Volumes/game/Applications/MATLAB_R2025b.app/bin/matlab
- 先验证建模链路（不运行仿真）：
  - setup_project
  - pmsm_foc_simscape(struct('run_sim', false))
  - 结果：MEX 编译成功，模型创建成功
- 再验证完整仿真链路：
  - setup_project
  - pmsm_foc_simscape
  - 语法修复后首次运行发现维度错误，按上面第 2 点修复
- 最后跑第一阶段验收：
  - setup_project
  - run_phase1_acceptance
  - 结果：
    - speed_step_only: PASS
    - speed_and_load_step: PASS
    - Acceptance result: PASS

当前状态说明：
- 第一阶段目标“C++ S-Function 在 Simscape 场景跑通”已满足。
- 仍有代数环告警（algebraic loop），但当前验收边界仅要求跑通，故不阻塞本阶段结论。
- 未引入 ROS2 闭环，未改变已确认边界与大方向。

### 3.5 本轮续修（2026-03-24，当天继续）
文件：
- matlab/s_function/sfun_pi_controller.cpp
- matlab/scripts/build_sfun_pid.m
- matlab/simscape/pmsm_foc_simscape.m
- matlab/simscape/create_pmsm_foc_model.m
- matlab/scripts/run_phase1_acceptance.m

背景：
- 用户要求控制器方案从“整块 FOC S-Function”转为“PID 的 S-Function（优先 C/C++）”。
- 在切换到 pid_sfun 后，run_phase1_acceptance 过程中出现维度错误：
  - pmsm_foc_model/FOC Controller/Mux_park 输出端口维度设置无效
  - pmsm_foc_model/FOC Controller/Park 输入端口 1 宽度为 2（期望不匹配）
  - 随后同类问题扩展到 Mux_abc/Clarke 链路

本轮已完成改动：
1) 新增 C++ PI S-Function 能力（控制器实现层）
- 新增 `sfun_pi_controller.cpp`，封装可复用 PI 控制器（基于 cpp/include/pid_controller.h）。
- 新增 `build_sfun_pid.m`，用于单独编译 PI S-Function MEX。

2) 主入口支持并默认使用 pid_sfun（调度层）
- `pmsm_foc_simscape.m` 新增 controller_mode 分支：
  - sfun -> build_sfun_foc
  - pid_sfun -> build_sfun_pid
- 默认控制器模式调整为 `controller_mode = pid_sfun`。

3) 模型生成器新增 plant_mode 分支（为转蓝库做接口准备）
- `create_pmsm_foc_model.m` 新增：
  - `plant_mode = code`（旧链路）
  - `plant_mode = simscape_blue`（蓝库链路）
- 在 `simscape_blue` 下新增 Model Reference 适配器：
  - 要求 `sim_params.simscape_plant_model` 显式提供
  - 固定 I/O 顺序：[duty_abc, Te_load] -> [ia, ib, ic, omega_m, Te, theta_m]
  - 未提供时“失败即报错”，避免静默回退到 code plant

4) 修复 pid_sfun 下维度错误（本轮关键修复）
- 将 pid_sfun 控制器中的向量化 `Mux_park + MATLAB Fcn(Park)` 替换为显式标量运算链：
  - sin/cos + product + sum 直接计算 id/iq
- 将向量化 `Mux_abc + MATLAB Fcn(Clarke)` 同步替换为显式 Gain/Sum 计算 i_alpha/i_beta
- 结果：消除 Mux_park/Park 与 Mux_abc/Clarke 的宽度推断错误

5) 验收回归脚本兼容
- `run_phase1_acceptance.m` 显式固定：
  - `controller_mode = pid_sfun`
  - `plant_mode = code`
- 目的是在转蓝库前，保持现有回归入口稳定可用。

本轮调试与验证（已实机执行）：
- MATLAB 可执行文件：`/Volumes/game/Applications/MATLAB_R2025b.app/bin/matlab`
- 验证命令：
  - setup_project
  - run_phase1_acceptance
- 结果：
  - speed_step_only: PASS
  - speed_and_load_step: PASS
  - Acceptance result: PASS
- 说明：仍有代数环告警，但不阻塞“先跑通”验收边界。

### 3.6 本轮续修（2026-03-25，转蓝库接线与稳定性修正）
文件：
- matlab/simscape/create_pmsm_foc_model.m
- matlab/simscape/pmsm_foc_simscape.m
- matlab/simscape/create_blue_plant_wrapper_template.m
- matlab/scripts/run_phase1_blue_acceptance.m
- README.md

背景：
- 用户已将闭源 Simscape 蓝库电机/逆变器接入 PlantCore，并确认旧接口 `[duty_abc, Te_load]` 不再匹配实际开关器件级模型。
- 实际蓝库需要的控制输入不再是三相占空比/调制定值，而是三相桥六路上下管门极信号。
- 初次改成 `gates_6` 后，虽然可进入仿真，但在真实蓝库模型下出现两类问题：
  - 代数环扩大到 Gate Driver 与 Model Reference 链路；
  - IGBT 零交叉事件在极短时间内连续触发，导致仿真停止。
- 随后用户反馈：MATLAB 端已经能够跑起来，但 `ia/ib/ic` 电流波形明显异常，说明“可运行”已达到，而“波形合理”仍未达到。

本轮已完成改动：
1) 蓝库输入接口从 `duty_abc` 扩展为可配置模式（接口层）
- `pmsm_foc_simscape.m` 新增 `sim_params.simscape_plant_input`：
  - `duty_abc`
  - `gates_6`
- 默认蓝库模式切到 `gates_6`，以适配真实开关器件模型。

2) 新增 Gate Driver：占空比 -> 六路门极（驱动层）
- 在 `create_pmsm_foc_model.m` 中为 `simscape_blue + gates_6` 路径新增 Gate Driver 子系统。
- 信号顺序固定为：
  - `[Sa+, Sa-, Sb+, Sb-, Sc+, Sc-]`
- 采用三相占空比与载波比较得到门极，并将 `boolean` 明确转换为 `double` 以匹配 Simscape Plant 输入。

3) 新增蓝库包装模型模板（适配层）
- 新增 `create_blue_plant_wrapper_template.m`，用于自动生成可被 Model Reference 引用的包装模型。
- 包装模型支持：
  - `duty_abc`
  - `gates_6`
- 默认模板已切到 `gates_6`，并修复了输入维度：
  - `duty_abc` 为 3 维
  - `gates_6` 为 6 维

4) 修复蓝库验收入口与路径问题（调度层）
- `run_phase1_blue_acceptance.m` 现在会自动补 MATLAB 路径，不再依赖手工 `addpath`。
- `pmsm_foc_simscape.m` 会自动将 `matlab/models` 加入 path，并尝试 `load_system` 被引用的蓝库包装模型。
- 对 `ee_lib/...` 这种库块路径增加失败即报错提示，避免误把库块路径当模型名传入。

5) 针对代数环与 ZC 风暴的稳定性修正（数值层）
- 在 Gate Driver 输入处增加一拍 `Unit Delay`：
  - 作用：打断“控制器输出 -> 开关网络 -> 测量 -> 控制器”的直接馈通环。
- 模型求解器设置增加：
  - `ZeroCrossAlgorithm = Adaptive`
  - `MaxConsecutiveZCs = 5000`
- 对引用蓝库模型尝试开启 `MinAlgLoopOccurrences = on`（若版本支持）。

6) 针对真实开关纹波的测量调理（本轮关键）
- 识别到蓝库场景下，控制器此前直接使用原始 `ia/ib/ic` 开关纹波电流做 Clarke/Park/PI，会显著恶化控制效果。
- 在 `create_measurement_subsystem` 中，为 `simscape_blue` 路径新增：
  - 控制周期同步 `Zero-Order Hold`
  - 电流一阶离散低通（默认 `current_meas_lpf_hz = 2000 Hz`）
  - 转速一阶离散低通（默认 `omega_meas_lpf_hz = 500 Hz`）
- 目标是让 FOC 使用“同步采样后的测量值”，而不是原始开关瞬态。

本轮调试与验证（已实机执行）：
- MATLAB 可执行文件：`/Volumes/game/Applications/MATLAB_R2025b.app/bin/matlab`
- 已验证命令：
  - `setup_project`
  - `create_blue_plant_wrapper_template('my_blue_plant_wrapper', false, 'gates_6')`
  - `run_phase1_blue_acceptance('my_blue_plant_wrapper', 'gates_6')`
  - `run_phase1_acceptance`
- 结果：
  - 蓝库模板链路（占位 PlantCore）：PASS
  - 旧 `code` plant 基线：PASS
- 对真实蓝库 PlantCore 的状态：
  - 仿真已能运行；
  - 但用户观察到 `ia/ib/ic` 波形明显不合理，说明当前主要矛盾已从“跑不起来”转为“波形质量/控制质量不对”。

经验总结：
1) 真实开关器件级 Simscape 植物不要直接复用平均值模型接口假设
- `duty_abc` 对平均值 inverter 适用，但真实逆变器通常需要 `gates_6`，接口层必须单独适配。

2) `Model Reference` 的输入必须是“模型名”，不是库块路径
- `ee_lib/...` 这类路径不能直接传给 `simscape_plant_model`。

3) 开关级模型下，原始相电流不能直接喂给 FOC
- 若不做同步采样/滤波，PI 控制会被 PWM 纹波、ZC 事件和开关瞬态强烈激励，表现为电流波形异常、代数环恶化甚至仿真中止。

4) “能跑通”不代表“控制正确”
- 本轮已经把问题推进到波形质量阶段；后续工作重心应从建模/接口问题转向控制调参与测量建模。

下一步明确建议：
1) 优先检查 `gates_6` 与蓝库逆变器门极顺序是否完全一致
- 当前代码约定顺序：`[Sa+, Sa-, Sb+, Sb-, Sc+, Sc-]`
- 若蓝库内部顺序不同，电流波形会严重异常。

2) 在真实蓝库模型上继续收紧控制器带宽
- 当前 `pid_sfun` 的 PI 参数来自平均值链路，未针对开关器件级模型重整定。
- 建议下一轮优先降低：
  - `Kp_id / Ki_id`
  - `Kp_iq / Ki_iq`
  - `Kp_speed / Ki_speed`

3) 继续完善门极驱动细节
- 当前 Gate Driver 还是“互补门极 + 一拍延迟”简化版本；
- 若真实蓝库逆变器对死区敏感，下一轮应增加 dead-time/non-overlap。

4) 将验收从“能跑通”扩展到“波形合理”
- 下一轮应至少增加以下观察量：
  - `ia/ib/ic` 包络是否平衡
  - `id/iq` 是否围绕参考收敛
  - 转速是否跟踪且不过度振荡
  - 占空比/门极是否长期打满

## 4. 本次未做但已识别的问题

- 蓝库 PlantCore 已可实际接入并跑通，但真实波形仍不合理：
  - 当前最高优先级已变为“控制质量与波形合理性”，不是“接口是否能连上”。
- 尚未确认真实蓝库逆变器的门极输入顺序是否与当前代码约定完全一致。
- 当前 `pid_sfun` 参数仍主要沿用平均值 plant 调参，尚未按开关器件级 Simscape plant 重新整定。
- 当前 Gate Driver 尚未加入 dead-time；若蓝库开关器件对互补直通或瞬态敏感，仍可能造成波形异常。
- 测量链虽已加入同步采样与低通，但尚未根据实际开关频率、电流纹波和传感器带宽做系统化整定。

## 5. 下一个 session 建议直接执行的流程（下一步重点：转蓝库）

### Step 1: 先做基线回归（确保当前链路稳定）
在 MATLAB 中执行：
- setup_project
- run_phase1_acceptance

收集并保存：
- 两个 case 的控制台输出
- 若失败，完整错误栈（包含报错 block 路径和求解器报错文本）

### Step 2: 若回归失败，按优先级排障
1) S-Function 构建与加载问题
- 检查 mex 编译器、include 路径、生成产物是否在 path。

2) 模型时序与求解器问题
- 检查控制采样时间、求解器设置、连续/离散块混用。

3) Simscape 接口问题
- 检查物理域到信号域转换、单位和量纲一致性。

4) 噪声引入稳定性问题
- 先将噪声设为 0 跑通，再逐步恢复。

### Step 3: 真实蓝库波形修正（最高优先）
1) 先核对门极顺序
- 核对 `PlantCore` 中逆变器 6 路门极输入顺序，是否与当前代码约定一致：
  - `[Sa+, Sa-, Sb+, Sb-, Sc+, Sc-]`

2) 观察控制内部量，不要只看相电流
- 在 MATLAB/Simulink 中同步查看：
  - `id_meas`
  - `iq_meas`
  - `speed`
  - `duty_abc` / `gates_6`
- 先判断是“电流采样异常”还是“控制器打满/发散”。

3) 重新整定 PI（优先降带宽）
- 在真实蓝库 plant 下优先尝试将当前 PI 参数整体下调到更保守水平，再做回归。

4) 若仍有异常，再补 Gate Driver 死区
- 为上下桥臂加入 dead-time/non-overlap，避免门极理想互补导致真实器件级模型过于敏感。

5) 完成两工况回归
- 在波形明显合理后，再回到：
  - `speed_step_only`
  - `speed_and_load_step`

## 6. 已确认的第一阶段边界（给下个 session 的硬约束）

- 只做 SPMSM。
- 只做 MATLAB/Simscape 闭环。
- 控制器主实现必须是 C++ S-Function。
- Python/ROS2 暂不纳入第一阶段闭环。
- 验收先看“跑通”，不看性能指标。
- 工况固定优先：速度阶跃、负载阶跃。

## 7. 建议下个 session 的首句 prompt

可直接复制给下个会话：

“请基于根目录 SESSION_HANDOFF_2026-03-24.md 继续推进第一阶段。先在 MATLAB 执行 run_phase1_acceptance 做基线验证；随后立刻推进转蓝库：将闭源 Simscape 开关器件模型接入 sim_params.simscape_plant_model，并切换 plant_mode=simscape_blue，在保持 C++ S-Function 控制器链路不变前提下跑通两工况。若失败，按交接文档排障优先级修复，不引入 ROS2 闭环。”

## 8. 本轮续补（2026-03-25，基线回归与波形导出）

文件：
- matlab/scripts/export_phase1_scope_images.m
- attachment/speed_step_only_phase_currents.png
- attachment/speed_step_only_dq_currents.png
- attachment/speed_step_only_omega_m.png
- attachment/speed_step_only_duty_abc.png
- attachment/speed_and_load_step_phase_currents.png
- attachment/speed_and_load_step_dq_currents.png
- attachment/speed_and_load_step_omega_m.png
- attachment/speed_and_load_step_duty_abc.png
- SESSION_HANDOFF_2026-03-24.md

背景：
- 用户要求先执行下一步计划中的 Step 1（基线回归），并给出最终结果。
- 随后用户要求将仿真后的“示波器内容”整理为图片，统一放到根目录 attachment 文件夹以便查看。

本轮已完成内容：
1) 完成 Step 1 基线回归（已实机执行）
- MATLAB 中执行：
  - setup_project
  - run_phase1_acceptance
- 结果：
  - speed_step_only: PASS
  - speed_and_load_step: PASS
  - Acceptance result: PASS
- 备注：
  - 两工况均可运行到 0.5 s；
  - 仍存在 1 个代数环告警（与先前状态一致），不阻塞当前“先跑通”验收边界。

2) 新增自动导出脚本（将关键波形导出为图片）
- 新增脚本：matlab/scripts/export_phase1_scope_images.m
- 脚本行为：
  - 自动运行两工况（speed_step_only / speed_and_load_step）；
  - 强制开启 sim_params.enable_debug_logging=true；
  - 从 sim_out 中读取 timeseries 日志：
    - ia_meas_ts, ib_meas_ts, ic_meas_ts
    - id_meas_ts, iq_meas_ts
    - omega_m_ts
    - duty_abc_ts
  - 导出 4 类图：
    - 相电流 ia/ib/ic
    - dq 电流 id/iq
    - 机械角速度 omega_m
    - 三相占空比 duty_abc
  - 默认输出目录：项目根目录 attachment/

3) 生成并落盘图片（已实机执行）
- 已调用 export_phase1_scope_images.m，确认 attachment 目录下生成 8 张 png：
  - speed_step_only_phase_currents.png
  - speed_step_only_dq_currents.png
  - speed_step_only_omega_m.png
  - speed_step_only_duty_abc.png
  - speed_and_load_step_phase_currents.png
  - speed_and_load_step_dq_currents.png
  - speed_and_load_step_omega_m.png
  - speed_and_load_step_duty_abc.png

调试记录（本轮一次修复）：
- 首次导出失败原因为脚本尝试从 base workspace 读取 ia_meas_ts 等变量。
- 实际日志位于 sim_out（Simulink.SimulationOutput）中。
- 已修复为从 sim_out.get(...) 取 timeseries 后重新执行，导出成功。

当前状态说明（截至本轮）：
- Step 1 基线回归已再次确认 PASS。
- 波形导出链路已打通，后续可一键复现并生成图片对比。
- 下一优先项仍是蓝库场景波形合理性修正（先核对 gates_6 门极顺序，再做 PI/测量/驱动细化）。
