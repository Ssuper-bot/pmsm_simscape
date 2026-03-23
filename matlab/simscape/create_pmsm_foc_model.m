function create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
%CREATE_PMSM_FOC_MODEL Programmatically create a fully-wired PMSM FOC model
%
% Creates a Simulink model with all blocks connected:
%   - PMSM motor (dq-frame equations via MATLAB Fcn)
%   - Average-model inverter (duty * Vdc)
%   - FOC controller (Clarke/Park/PI/InvPark/SVPWM, inline equations)
%   - Measurement subsystem (pass-through + theta_m->theta_e)
%   - Mechanical load (step torque)
%   - Reference generator (speed ramp + id_ref=0)
%   - Scopes

    %% Create new model
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    new_system(model_name);
    open_system(model_name);

    % Set solver
    set_param(model_name, ...
        'Solver', sim_params.solver, ...
        'StopTime', num2str(sim_params.t_end), ...
        'MaxStep', num2str(sim_params.max_step), ...
        'RelTol', '1e-4', ...
        'AbsTol', '1e-6');

    %% ===== Create Subsystem Blocks =====

    % --- Reference Generator ---
    ref_gen = [model_name '/Reference Generator'];
    add_block('built-in/Subsystem', ref_gen);
    create_reference_generator(ref_gen, ref_params);
    set_param(ref_gen, 'Position', [50 30 180 110]);

    % --- FOC Controller Subsystem ---
    foc_ctrl = [model_name '/FOC Controller'];
    add_block('built-in/Subsystem', foc_ctrl);
    create_foc_controller_subsystem(foc_ctrl, ctrl_params, inv_params);
    set_param(foc_ctrl, 'Position', [50 180 250 530]);

    % --- Power Stage Subsystem ---
    power_stage = [model_name '/Power Stage'];
    add_block('built-in/Subsystem', power_stage);
    create_power_stage(power_stage, inv_params);
    set_param(power_stage, 'Position', [350 200 500 380]);

    % --- PMSM Motor Block (dq-frame placeholder) ---
    pmsm_block = [model_name '/PMSM Motor'];
    add_block('built-in/Subsystem', pmsm_block);
    create_pmsm_subsystem(pmsm_block, motor_params, sim_params);
    set_param(pmsm_block, 'Position', [600 180 780 400]);

    % --- Mechanical Load ---
    load_block = [model_name '/Mechanical Load'];
    add_block('built-in/Subsystem', load_block);
    create_mechanical_load(load_block, ref_params);
    set_param(load_block, 'Position', [880 220 1020 340]);

    % --- Measurements ---
    meas_block = [model_name '/Measurements'];
    add_block('built-in/Subsystem', meas_block);
    create_measurement_subsystem(meas_block, motor_params);
    set_param(meas_block, 'Position', [600 460 780 620]);

    % --- Scopes ---
    create_scopes(model_name);

    %% ===== Connect Top-Level Blocks =====

    % Reference Generator -> FOC Controller
    add_line(model_name, 'Reference Generator/1', 'FOC Controller/6', 'autorouting', 'smart');
    add_line(model_name, 'Reference Generator/2', 'FOC Controller/7', 'autorouting', 'smart');

    % FOC Controller -> Power Stage  (duty_abc is 3-element vector)
    add_line(model_name, 'FOC Controller/1', 'Power Stage/1', 'autorouting', 'smart');

    % Power Stage -> PMSM Motor  (Va, Vb, Vc)
    add_line(model_name, 'Power Stage/1', 'PMSM Motor/1', 'autorouting', 'smart');
    add_line(model_name, 'Power Stage/2', 'PMSM Motor/2', 'autorouting', 'smart');
    add_line(model_name, 'Power Stage/3', 'PMSM Motor/3', 'autorouting', 'smart');

    % Mechanical Load -> PMSM Motor  (Te_load)
    add_line(model_name, 'Mechanical Load/1', 'PMSM Motor/4', 'autorouting', 'smart');

    % PMSM Motor -> Measurements  (ia, ib, ic, theta_m, omega_m)
    add_line(model_name, 'PMSM Motor/1', 'Measurements/1', 'autorouting', 'smart');
    add_line(model_name, 'PMSM Motor/2', 'Measurements/2', 'autorouting', 'smart');
    add_line(model_name, 'PMSM Motor/3', 'Measurements/3', 'autorouting', 'smart');
    add_line(model_name, 'PMSM Motor/6', 'Measurements/4', 'autorouting', 'smart');
    add_line(model_name, 'PMSM Motor/4', 'Measurements/5', 'autorouting', 'smart');

    % Measurements -> FOC Controller  (ia, ib, ic, theta_e, omega_m)
    add_line(model_name, 'Measurements/1', 'FOC Controller/1', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/2', 'FOC Controller/2', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/3', 'FOC Controller/3', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/4', 'FOC Controller/4', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/5', 'FOC Controller/5', 'autorouting', 'smart');

    % Scopes
    add_line(model_name, 'PMSM Motor/4', 'Speed Scope/1', 'autorouting', 'smart');
    add_line(model_name, 'FOC Controller/2', 'Current Scope/1', 'autorouting', 'smart');
    add_line(model_name, 'PMSM Motor/5', 'Torque Scope/1', 'autorouting', 'smart');

    %% Annotation
    try
        ann = Simulink.Annotation([model_name '/PMSM FOC Model - Auto-generated']);
        ann.Position = [400 650];
    catch
    end

    %% Save model
    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end
    save_system(model_name, fullfile(model_dir, [model_name '.slx']));
    fprintf('Model saved to: %s\n', fullfile(model_dir, [model_name '.slx']));
end

%% ===================== Helper Functions =====================

function create_power_stage(subsys, inv_params)
%CREATE_POWER_STAGE Average-model three-phase inverter
%   Input 1: duty_abc [3x1]
%   Output 1: Va  Output 2: Vb  Output 3: Vc

    % Ports
    add_block('built-in/Inport',  [subsys '/duty_abc'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/Va'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/Vb'], 'Port', '2');
    add_block('built-in/Outport', [subsys '/Vc'], 'Port', '3');

    % Vdc as constant (no external input port - avoids dimension issues)
    add_block('built-in/Constant', [subsys '/Vdc'], ...
        'Value', num2str(inv_params.Vdc), ...
        'Position', [80 150 130 170]);

    % Element-wise product: duty_abc .* Vdc (scalar expansion)
    add_block('simulink/Math Operations/Product', [subsys '/Modulator'], ...
        'Inputs', '2', ...
        'Multiplication', 'Element-wise(.*)', ...
        'Position', [200 80 250 130]);

    % Demux 3-element vector -> scalars
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux'], ...
        'Outputs', '3', ...
        'Position', [300 70 310 150]);

    % Wiring
    add_line(subsys, 'duty_abc/1', 'Modulator/1');
    add_line(subsys, 'Vdc/1', 'Modulator/2');
    add_line(subsys, 'Modulator/1', 'Demux/1');
    add_line(subsys, 'Demux/1', 'Va/1');
    add_line(subsys, 'Demux/2', 'Vb/1');
    add_line(subsys, 'Demux/3', 'Vc/1');
end

function create_foc_controller_subsystem(subsys, ctrl_params, inv_params)
%CREATE_FOC_CONTROLLER_SUBSYSTEM Fully-wired FOC controller
%   Inputs:  1:ia  2:ib  3:ic  4:theta_e  5:omega_m  6:speed_ref  7:id_ref
%   Outputs: 1:duty_abc[3x1]  2:id_meas  3:iq_meas

    % === Input Ports ===
    add_block('built-in/Inport', [subsys '/ia'],        'Port', '1');
    add_block('built-in/Inport', [subsys '/ib'],        'Port', '2');
    add_block('built-in/Inport', [subsys '/ic'],        'Port', '3');
    add_block('built-in/Inport', [subsys '/theta_e'],   'Port', '4');
    add_block('built-in/Inport', [subsys '/omega_m'],   'Port', '5');
    add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
    add_block('built-in/Inport', [subsys '/id_ref'],    'Port', '7');

    % === Output Ports ===
    add_block('built-in/Outport', [subsys '/duty_abc'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/id_meas'],  'Port', '2');
    add_block('built-in/Outport', [subsys '/iq_meas'],  'Port', '3');

    % ---- Clarke Transform: [ia,ib,ic] -> [i_alpha, i_beta] ----
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_abc'], ...
        'Inputs', '3', 'Position', [120 60 125 140]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/Clarke'], ...
        'MATLABFcn', '[2/3*(u(1)-0.5*u(2)-0.5*u(3)); 2/3*(sqrt(3)/2*u(2)-sqrt(3)/2*u(3))]', ...
        'Position', [170 80 290 120]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_ab'], ...
        'Outputs', '2', 'Position', [320 80 325 120]);

    add_line(subsys, 'ia/1', 'Mux_abc/1');
    add_line(subsys, 'ib/1', 'Mux_abc/2');
    add_line(subsys, 'ic/1', 'Mux_abc/3');
    add_line(subsys, 'Mux_abc/1', 'Clarke/1');
    add_line(subsys, 'Clarke/1', 'Demux_ab/1');

    % ---- Park Transform: [i_alpha, i_beta, theta_e] -> [id, iq] ----
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_park'], ...
        'Inputs', '3', 'Position', [370 70 375 150]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/Park'], ...
        'MATLABFcn', '[u(1)*cos(u(3))+u(2)*sin(u(3)); -u(1)*sin(u(3))+u(2)*cos(u(3))]', ...
        'Position', [410 90 530 130]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_dq'], ...
        'Outputs', '2', 'Position', [560 90 565 130]);

    add_line(subsys, 'Demux_ab/1', 'Mux_park/1');
    add_line(subsys, 'Demux_ab/2', 'Mux_park/2');
    add_line(subsys, 'theta_e/1', 'Mux_park/3');
    add_line(subsys, 'Mux_park/1', 'Park/1');
    add_line(subsys, 'Park/1', 'Demux_dq/1');

    % Output measured id, iq
    add_line(subsys, 'Demux_dq/1', 'id_meas/1');
    add_line(subsys, 'Demux_dq/2', 'iq_meas/1');

    % ---- Speed PI: speed_ref - omega_m -> iq_ref ----
    % Convert speed_ref from RPM to rad/s first
    add_block('simulink/Math Operations/Gain', [subsys '/RPM2RadS'], ...
        'Gain', '2*pi/60', ...
        'Position', [80 200 120 220]);
    add_block('simulink/Math Operations/Sum', [subsys '/Speed Error'], ...
        'Inputs', '+-', 'Position', [160 200 190 230]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Speed PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_speed), ...
        'I', num2str(ctrl_params.Ki_speed), ...
        'Position', [230 200 310 240]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Iq Sat'], ...
        'UpperLimit', num2str(ctrl_params.iq_max), ...
        'LowerLimit', num2str(-ctrl_params.iq_max), ...
        'Position', [350 200 400 240]);

    add_line(subsys, 'speed_ref/1', 'RPM2RadS/1');
    add_line(subsys, 'RPM2RadS/1', 'Speed Error/1');
    add_line(subsys, 'omega_m/1', 'Speed Error/2');
    add_line(subsys, 'Speed Error/1', 'Speed PI/1');
    add_line(subsys, 'Speed PI/1', 'Iq Sat/1');

    % ---- d-axis Current PI: id_ref - id -> vd ----
    add_block('simulink/Math Operations/Sum', [subsys '/Id Error'], ...
        'Inputs', '+-', 'Position', [600 160 630 190]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Id PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_id), ...
        'I', num2str(ctrl_params.Ki_id), ...
        'Position', [670 160 750 200]);

    add_line(subsys, 'id_ref/1', 'Id Error/1');
    add_line(subsys, 'Demux_dq/1', 'Id Error/2');
    add_line(subsys, 'Id Error/1', 'Id PI/1');

    % ---- q-axis Current PI: iq_ref - iq -> vq ----
    add_block('simulink/Math Operations/Sum', [subsys '/Iq Error'], ...
        'Inputs', '+-', 'Position', [600 250 630 280]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Iq PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_iq), ...
        'I', num2str(ctrl_params.Ki_iq), ...
        'Position', [670 250 750 290]);

    add_line(subsys, 'Iq Sat/1', 'Iq Error/1');
    add_line(subsys, 'Demux_dq/2', 'Iq Error/2');
    add_line(subsys, 'Iq Error/1', 'Iq PI/1');

    % ---- Inverse Park: [vd, vq, theta_e] -> [v_alpha, v_beta] ----
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_invpark'], ...
        'Inputs', '3', 'Position', [800 180 805 280]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/InvPark'], ...
        'MATLABFcn', '[u(1)*cos(u(3))-u(2)*sin(u(3)); u(1)*sin(u(3))+u(2)*cos(u(3))]', ...
        'Position', [840 210 960 250]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_vab'], ...
        'Outputs', '2', 'Position', [990 210 995 250]);

    add_line(subsys, 'Id PI/1', 'Mux_invpark/1');
    add_line(subsys, 'Iq PI/1', 'Mux_invpark/2');
    add_line(subsys, 'theta_e/1', 'Mux_invpark/3');
    add_line(subsys, 'Mux_invpark/1', 'InvPark/1');
    add_line(subsys, 'InvPark/1', 'Demux_vab/1');

    % ---- SVPWM: [v_alpha, v_beta, Vdc] -> [da; db; dc] ----
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_svpwm'], ...
        'Inputs', '3', 'Position', [1030 200 1035 270]);
    add_block('built-in/Constant', [subsys '/Vdc_const'], ...
        'Value', num2str(inv_params.Vdc), ...
        'Position', [980 270 1020 290]);

    % Inline SVPWM (avoids package namespace issues)
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/SVPWM'], ...
        'MATLABFcn', 'svpwm_inline(u(1), u(2), u(3))', ...
        'Position', [1070 210 1180 250]);

    add_line(subsys, 'Demux_vab/1', 'Mux_svpwm/1');
    add_line(subsys, 'Demux_vab/2', 'Mux_svpwm/2');
    add_line(subsys, 'Vdc_const/1', 'Mux_svpwm/3');
    add_line(subsys, 'Mux_svpwm/1', 'SVPWM/1');
    add_line(subsys, 'SVPWM/1', 'duty_abc/1');
end

function create_pmsm_subsystem(subsys, motor_params, sim_params)
%CREATE_PMSM_SUBSYSTEM PMSM motor using integrator-based state equations
%   Inputs:  1:Va  2:Vb  3:Vc  4:Te_load
%   Outputs: 1:ia  2:ib  3:ic  4:omega_m  5:Te  6:theta_m

    % --- Input ports ---
    add_block('built-in/Inport', [subsys '/Va'],      'Port', '1');
    add_block('built-in/Inport', [subsys '/Vb'],      'Port', '2');
    add_block('built-in/Inport', [subsys '/Vc'],      'Port', '3');
    add_block('built-in/Inport', [subsys '/Te_load'], 'Port', '4');

    % --- Output ports ---
    add_block('built-in/Outport', [subsys '/ia'],      'Port', '1');
    add_block('built-in/Outport', [subsys '/ib'],      'Port', '2');
    add_block('built-in/Outport', [subsys '/ic'],      'Port', '3');
    add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '4');
    add_block('built-in/Outport', [subsys '/Te'],      'Port', '5');
    add_block('built-in/Outport', [subsys '/theta_m'], 'Port', '6');

    % --- Mux all inputs [Va; Vb; Vc; Te_load] ---
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_in'], ...
        'Inputs', '4', 'Position', [100 80 105 230]);

    add_line(subsys, 'Va/1', 'Mux_in/1');
    add_line(subsys, 'Vb/1', 'Mux_in/2');
    add_line(subsys, 'Vc/1', 'Mux_in/3');
    add_line(subsys, 'Te_load/1', 'Mux_in/4');

    % --- Combine exogenous inputs (4) with state feedback (4) ---
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_all'], ...
        'Inputs', '2', 'Position', [200 100 205 200]);

    % Build the inline function string with motor parameters baked in
    Rs_s  = num2str(motor_params.Rs, '%.6g');
    Ld_s  = num2str(motor_params.Ld, '%.6g');
    Lq_s  = num2str(motor_params.Lq, '%.6g');
    fp_s  = num2str(motor_params.flux_pm, '%.6g');
    pp_s  = num2str(motor_params.p);
    J_s   = num2str(motor_params.J, '%.6g');
    B_s   = num2str(motor_params.B, '%.6g');

    fcn_str = ['pmsm_model_fcn(u(1),u(2),u(3),u(4),u(5),u(6),u(7),u(8),' ...
               Rs_s ',' Ld_s ',' Lq_s ',' fp_s ',' pp_s ',' J_s ',' B_s ')'];

    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/PMSM_Eqn'], ...
        'MATLABFcn', fcn_str, ...
        'Position', [250 120 420 180]);

    % Demux output: [did_dt; diq_dt; dwm_dt; dtheta_dt; ia; ib; ic; Te]
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_out'], ...
        'Outputs', '8', 'Position', [460 80 465 260]);

    add_line(subsys, 'Mux_in/1', 'Mux_all/1');
    add_line(subsys, 'Mux_all/1', 'PMSM_Eqn/1');
    add_line(subsys, 'PMSM_Eqn/1', 'Demux_out/1');

    % --- Integrators for the 4 states: id, iq, omega_m, theta_m ---
    states = {'id', 'iq', 'wm', 'theta'};
    y_pos = [80, 120, 160, 200];
    for k = 1:4
        blk = [subsys '/Int_' states{k}];
        add_block('simulink/Continuous/Integrator', blk, ...
            'Position', [530 y_pos(k) 570 y_pos(k)+20]);
        add_line(subsys, ['Demux_out/' num2str(k)], ['Int_' states{k} '/1']);
    end

    % Mux the 4 integrated states back -> feedback
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_states'], ...
        'Inputs', '4', 'Position', [620 80 625 220]);
    for k = 1:4
        add_line(subsys, ['Int_' states{k} '/1'], ['Mux_states/' num2str(k)]);
    end
    add_line(subsys, 'Mux_states/1', 'Mux_all/2');

    % --- Wire outputs ---
    add_line(subsys, 'Demux_out/5', 'ia/1');
    add_line(subsys, 'Demux_out/6', 'ib/1');
    add_line(subsys, 'Demux_out/7', 'ic/1');
    add_line(subsys, 'Int_wm/1',    'omega_m/1');
    add_line(subsys, 'Demux_out/8', 'Te/1');
    add_line(subsys, 'Int_theta/1', 'theta_m/1');
end

function create_measurement_subsystem(subsys, motor_params)
%CREATE_MEASUREMENT_SUBSYSTEM Pass-through currents + theta_e + omega_m
%   Inputs:  1:ia  2:ib  3:ic  4:theta_m  5:omega_m
%   Outputs: 1:ia_meas  2:ib_meas  3:ic_meas  4:theta_e  5:omega_m

    add_block('built-in/Inport',  [subsys '/ia'],      'Port', '1');
    add_block('built-in/Inport',  [subsys '/ib'],      'Port', '2');
    add_block('built-in/Inport',  [subsys '/ic'],      'Port', '3');
    add_block('built-in/Inport',  [subsys '/theta_m'], 'Port', '4');
    add_block('built-in/Inport',  [subsys '/omega_m_in'], 'Port', '5');
    add_block('built-in/Outport', [subsys '/ia_meas'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/ib_meas'], 'Port', '2');
    add_block('built-in/Outport', [subsys '/ic_meas'], 'Port', '3');
    add_block('built-in/Outport', [subsys '/theta_e'], 'Port', '4');
    add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '5');

    % Pass-through currents
    add_line(subsys, 'ia/1', 'ia_meas/1');
    add_line(subsys, 'ib/1', 'ib_meas/1');
    add_line(subsys, 'ic/1', 'ic_meas/1');

    % theta_e = pole_pairs * theta_m
    add_block('simulink/Math Operations/Gain', [subsys '/PoleP'], ...
        'Gain', num2str(motor_params.p), ...
        'Position', [200 180 250 210]);
    add_line(subsys, 'theta_m/1', 'PoleP/1');
    add_line(subsys, 'PoleP/1', 'theta_e/1');

    % omega_m: direct pass-through (no Derivative block - avoids noise)
    add_line(subsys, 'omega_m_in/1', 'omega_m/1');
end

function create_mechanical_load(subsys, ref_params)
%CREATE_MECHANICAL_LOAD Step torque load
%   Output 1: Te_load

    add_block('built-in/Outport', [subsys '/Te_load'], 'Port', '1');

    add_block('simulink/Sources/Step', [subsys '/Load Step'], ...
        'Time', num2str(ref_params.load_step_time), ...
        'Before', '0', ...
        'After', num2str(ref_params.load_torque), ...
        'Position', [80 60 130 90]);

    add_line(subsys, 'Load Step/1', 'Te_load/1');
end

function create_reference_generator(subsys, ref_params)
%CREATE_REFERENCE_GENERATOR Speed ramp + id_ref=0
%   Output 1: speed_ref (RPM, ramped)  Output 2: id_ref

    add_block('built-in/Outport', [subsys '/speed_ref'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/id_ref'],    'Port', '2');

    add_block('simulink/Sources/Ramp', [subsys '/Speed Ramp'], ...
        'Slope', num2str(ref_params.speed_ref / ref_params.speed_ramp_time), ...
        'Position', [50 30 120 60]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Speed Sat'], ...
        'UpperLimit', num2str(ref_params.speed_ref), ...
        'LowerLimit', '0', ...
        'Position', [160 30 220 60]);
    add_block('built-in/Constant', [subsys '/Id Ref'], ...
        'Value', num2str(ref_params.id_ref), ...
        'Position', [100 90 150 110]);

    add_line(subsys, 'Speed Ramp/1', 'Speed Sat/1');
    add_line(subsys, 'Speed Sat/1', 'speed_ref/1');
    add_line(subsys, 'Id Ref/1', 'id_ref/1');
end

function create_scopes(model_name)
%CREATE_SCOPES Visualization scopes
    add_block('simulink/Sinks/Scope', [model_name '/Speed Scope'], ...
        'Position', [1100 50 1160 100]);
    add_block('simulink/Sinks/Scope', [model_name '/Current Scope'], ...
        'Position', [1100 180 1160 230]);
    add_block('simulink/Sinks/Scope', [model_name '/Torque Scope'], ...
        'Position', [1100 310 1160 360]);
end
