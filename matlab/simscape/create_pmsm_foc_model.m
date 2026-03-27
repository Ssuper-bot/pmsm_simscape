function create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
%CREATE_PMSM_FOC_MODEL Programmatically create a fully-wired PMSM FOC model
%
% Creates a Simulink model with all blocks connected:
%   - Plant path A: legacy code-based inverter + motor equations
%   - Plant path B: Simscape blue-library plant via model reference
%   - FOC controller (Clarke/Park/PI/InvPark/SVPWM, inline equations)
%   - Measurement subsystem (pass-through + theta_m->theta_e)
%   - Signals input subsystem (reference generator + mechanical load)
%   - Scopes

    %% Create new model
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    new_system(model_name);
    open_system(model_name);

    % Set solver
    set_param(model_name, ...
        'SolverType', 'Variable-step', ...
        'Solver', sim_params.solver, ...
        'StopTime', num2str(sim_params.t_end), ...
        'MaxStep', num2str(sim_params.max_step), ...
        'RelTol', '1e-4', ...
        'AbsTol', '1e-6', ...
        'ZeroCrossAlgorithm', 'Adaptive', ...
        'MaxConsecutiveZCs', '5000');

    %% ===== Create Subsystem Blocks =====

    plant_mode = 'code';
    plant_input_mode = 'duty_abc';
    if isfield(sim_params, 'plant_mode')
        plant_mode = sim_params.plant_mode;
    end
    if isfield(sim_params, 'simscape_plant_input') && ~isempty(sim_params.simscape_plant_input)
        plant_input_mode = sim_params.simscape_plant_input;
    end

    % --- Signals input module (reference + load) ---
    signals_block = [model_name '/signals'];
    add_block('built-in/Subsystem', signals_block);
    create_signals_subsystem(signals_block, ref_params, motor_params, ctrl_params, sim_params);
    set_param(signals_block, 'Position', [40 30 260 260]);

    % --- FOC Controller Subsystem ---
    foc_ctrl = [model_name '/FOC Controller'];
    add_block('built-in/Subsystem', foc_ctrl);
    create_foc_controller_subsystem(foc_ctrl, ctrl_params, inv_params, motor_params, sim_params);
    set_param(foc_ctrl, 'Position', [50 180 250 530]);

    if strcmpi(plant_mode, 'simscape_blue')
        if strcmpi(plant_input_mode, 'gates_6')
            gate_driver = [model_name '/Gate Driver'];
            add_block('built-in/Subsystem', gate_driver);
            create_gate_driver_subsystem(gate_driver, inv_params, sim_params);
            set_param(gate_driver, 'Position', [280 200 380 360]);
        end

        plant_block = [model_name '/Simscape Plant'];
        add_block('built-in/Subsystem', plant_block);
        create_simscape_blue_plant(plant_block, sim_params);
        set_param(plant_block, 'Position', [450 180 690 420]);
    else
        % --- Legacy Power Stage Subsystem ---
        power_stage = [model_name '/Power Stage'];
        add_block('built-in/Subsystem', power_stage);
        create_power_stage(power_stage, inv_params);
        set_param(power_stage, 'Position', [350 200 500 380]);

        % --- Legacy PMSM Motor Block (dq-frame equations) ---
        pmsm_block = [model_name '/PMSM Motor'];
        add_block('built-in/Subsystem', pmsm_block);
        create_pmsm_subsystem(pmsm_block, motor_params, sim_params);
        set_param(pmsm_block, 'Position', [600 180 780 400]);
    end

    % --- Measurements ---
    meas_block = [model_name '/Measurements'];
    add_block('built-in/Subsystem', meas_block);
    create_measurement_subsystem(meas_block, motor_params, sim_params);
    set_param(meas_block, 'Position', [600 460 780 620]);

    % --- Scopes ---
    create_scopes(model_name);

    %% ===== Connect Top-Level Blocks =====

    % signals -> FOC Controller
    add_line(model_name, 'signals/1', 'FOC Controller/6', 'autorouting', 'smart');
    add_line(model_name, 'signals/2', 'FOC Controller/7', 'autorouting', 'smart');
    add_line(model_name, 'signals/3', 'FOC Controller/8', 'autorouting', 'smart');

    if strcmpi(plant_mode, 'simscape_blue')
        if strcmpi(plant_input_mode, 'gates_6')
            % FOC Controller duty_abc -> gate pulses -> Plant input
            add_line(model_name, 'FOC Controller/1', 'Gate Driver/1', 'autorouting', 'smart');
            add_line(model_name, 'Gate Driver/1', 'Simscape Plant/1', 'autorouting', 'smart');
        else
            % FOC Controller duty_abc -> Plant input
            add_line(model_name, 'FOC Controller/1', 'Simscape Plant/1', 'autorouting', 'smart');
        end

        % signals torque command -> Plant torque input
        add_line(model_name, 'signals/4', 'Simscape Plant/2', 'autorouting', 'smart');

        % Plant -> Measurements (ia, ib, ic, theta_m, omega_m)
        add_line(model_name, 'Simscape Plant/1', 'Measurements/1', 'autorouting', 'smart');
        add_line(model_name, 'Simscape Plant/2', 'Measurements/2', 'autorouting', 'smart');
        add_line(model_name, 'Simscape Plant/3', 'Measurements/3', 'autorouting', 'smart');
        add_line(model_name, 'Simscape Plant/6', 'Measurements/4', 'autorouting', 'smart');
        add_line(model_name, 'Simscape Plant/4', 'Measurements/5', 'autorouting', 'smart');
    else
        % FOC Controller -> Power Stage  (duty_abc is 3-element vector)
        add_line(model_name, 'FOC Controller/1', 'Power Stage/1', 'autorouting', 'smart');

        % Power Stage -> PMSM Motor  (Va, Vb, Vc)
        add_line(model_name, 'Power Stage/1', 'PMSM Motor/1', 'autorouting', 'smart');
        add_line(model_name, 'Power Stage/2', 'PMSM Motor/2', 'autorouting', 'smart');
        add_line(model_name, 'Power Stage/3', 'PMSM Motor/3', 'autorouting', 'smart');

        % signals torque command -> PMSM Motor  (Te_load)
        add_line(model_name, 'signals/4', 'PMSM Motor/4', 'autorouting', 'smart');

        % PMSM Motor -> Measurements  (ia, ib, ic, theta_m, omega_m)
        add_line(model_name, 'PMSM Motor/1', 'Measurements/1', 'autorouting', 'smart');
        add_line(model_name, 'PMSM Motor/2', 'Measurements/2', 'autorouting', 'smart');
        add_line(model_name, 'PMSM Motor/3', 'Measurements/3', 'autorouting', 'smart');
        add_line(model_name, 'PMSM Motor/6', 'Measurements/4', 'autorouting', 'smart');
        add_line(model_name, 'PMSM Motor/4', 'Measurements/5', 'autorouting', 'smart');
    end

    % Measurements -> FOC Controller  (ia, ib, ic, theta_e, omega_m)
    add_line(model_name, 'Measurements/1', 'FOC Controller/1', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/2', 'FOC Controller/2', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/3', 'FOC Controller/3', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/4', 'FOC Controller/4', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/5', 'FOC Controller/5', 'autorouting', 'smart');

    % Scopes compare input refs (From tags from signals) vs measured outputs.
    wire_scope_compare_paths(model_name, plant_mode);

    if isfield(sim_params, 'enable_debug_logging') && logical(sim_params.enable_debug_logging)
        create_debug_logging(model_name, plant_mode, plant_input_mode);
    end

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

function create_simscape_blue_plant(subsys, sim_params)
%CREATE_SIMSCAPE_BLUE_PLANT Connect Simscape blue-library plant via model reference.
%   Input 1: duty_abc [3x1] OR gates_6 [6x1]
%   Input 2: Te_load
%   Output 1: ia
%   Output 2: ib
%   Output 3: ic
%   Output 4: omega_m
%   Output 5: Te
%   Output 6: theta_m

    if ~isfield(sim_params, 'simscape_plant_model') || isempty(sim_params.simscape_plant_model)
        error(['plant_mode=simscape_blue requires sim_params.simscape_plant_model.\n' ...
               'Expected model I/O order: [plant_cmd, Te_load] -> [ia, ib, ic, omega_m, Te, theta_m].']);
    end

    plant_model = sim_params.simscape_plant_model;
    plant_input_mode = 'duty_abc';
    if isfield(sim_params, 'simscape_plant_input') && ~isempty(sim_params.simscape_plant_input)
        plant_input_mode = sim_params.simscape_plant_input;
    end

    if strcmpi(plant_input_mode, 'gates_6')
        add_block('built-in/Inport',  [subsys '/gates_6'], 'Port', '1', 'PortDimensions', '6');
        cmd_port = 'gates_6/1';
    elseif strcmpi(plant_input_mode, 'duty_abc')
        add_block('built-in/Inport',  [subsys '/duty_abc'], 'Port', '1', 'PortDimensions', '3');
        cmd_port = 'duty_abc/1';
    else
        error('Unsupported simscape_plant_input: %s. Use duty_abc or gates_6.', plant_input_mode);
    end

    add_block('built-in/Inport',  [subsys '/Te_load'],  'Port', '2');
    add_block('built-in/Outport', [subsys '/ia'],       'Port', '1');
    add_block('built-in/Outport', [subsys '/ib'],       'Port', '2');
    add_block('built-in/Outport', [subsys '/ic'],       'Port', '3');
    add_block('built-in/Outport', [subsys '/omega_m'],  'Port', '4');
    add_block('built-in/Outport', [subsys '/Te'],       'Port', '5');
    add_block('built-in/Outport', [subsys '/theta_m'],  'Port', '6');

    add_block('simulink/Ports & Subsystems/Model', [subsys '/PlantRef'], ...
        'ModelName', plant_model, ...
        'Position', [150 80 330 260]);

    add_line(subsys, cmd_port, 'PlantRef/1');
    add_line(subsys, 'Te_load/1', 'PlantRef/2');
    add_line(subsys, 'PlantRef/1', 'ia/1');
    add_line(subsys, 'PlantRef/2', 'ib/1');
    add_line(subsys, 'PlantRef/3', 'ic/1');
    add_line(subsys, 'PlantRef/4', 'omega_m/1');
    add_line(subsys, 'PlantRef/5', 'Te/1');
    add_line(subsys, 'PlantRef/6', 'theta_m/1');
end

function create_gate_driver_subsystem(subsys, inv_params, sim_params)
%CREATE_GATE_DRIVER_SUBSYSTEM Convert duty_abc to 6 complementary gate pulses.
%   Input 1: duty_abc [3x1]
%   Output 1: gates_6 [6x1] = [Sa+ Sa- Sb+ Sb- Sc+ Sc-]

    gate_order = 1:6;
    if isfield(sim_params, 'gates_order') && ~isempty(sim_params.gates_order)
        candidate = sim_params.gates_order;
        if isnumeric(candidate) && numel(candidate) == 6 && all(sort(candidate(:)') == 1:6)
            gate_order = double(candidate(:)');
        else
            error('sim_params.gates_order must be a permutation of [1 2 3 4 5 6].');
        end
    end

    add_block('built-in/Inport', [subsys '/duty_abc'], 'Port', '1', 'PortDimensions', '3');
    add_block('built-in/Outport', [subsys '/gates_6'], 'Port', '1', 'PortDimensions', '6');

    % One control-tick delay breaks direct feedthrough algebraic loop
    % between controller output and switching-network currents.
    add_block('simulink/Discrete/Unit Delay', [subsys '/DutyDelay'], ...
        'SampleTime', num2str(sim_params.Ts_control), ...
        'InitialCondition', '[0.5 0.5 0.5]', ...
        'Position', [50 30 95 70]);

    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_duty'], ...
        'Outputs', '3', 'Position', [70 75 75 175]);

    add_block('simulink/Sources/Repeating Sequence', [subsys '/Carrier'], ...
        'rep_seq_t', sprintf('[0 %.12g]', 1 / inv_params.fsw), ...
        'rep_seq_y', '[0 1]', ...
        'Position', [120 210 210 240]);

    add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/CmpA'], ...
        'Operator', '>=', 'Position', [170 50 220 80]);
    add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/CmpB'], ...
        'Operator', '>=', 'Position', [170 100 220 130]);
    add_block('simulink/Logic and Bit Operations/Relational Operator', [subsys '/CmpC'], ...
        'Operator', '>=', 'Position', [170 150 220 180]);

    add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/NotA'], ...
        'Operator', 'NOT', 'Position', [255 50 290 80]);
    add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/NotB'], ...
        'Operator', 'NOT', 'Position', [255 100 290 130]);
    add_block('simulink/Logic and Bit Operations/Logical Operator', [subsys '/NotC'], ...
        'Operator', 'NOT', 'Position', [255 150 290 180]);

    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_gates'], ...
        'Inputs', '6', 'Position', [330 60 335 190]);
    add_block('simulink/Signal Attributes/Data Type Conversion', [subsys '/GatesToDouble'], ...
        'OutDataTypeStr', 'double', ...
        'Position', [370 105 440 145]);

    add_line(subsys, 'duty_abc/1', 'DutyDelay/1');
    add_line(subsys, 'DutyDelay/1', 'Demux_duty/1');
    add_line(subsys, 'Demux_duty/1', 'CmpA/1');
    add_line(subsys, 'Demux_duty/2', 'CmpB/1');
    add_line(subsys, 'Demux_duty/3', 'CmpC/1');

    add_line(subsys, 'Carrier/1', 'CmpA/2');
    add_line(subsys, 'Carrier/1', 'CmpB/2');
    add_line(subsys, 'Carrier/1', 'CmpC/2');

    add_line(subsys, 'CmpA/1', 'NotA/1');
    add_line(subsys, 'CmpB/1', 'NotB/1');
    add_line(subsys, 'CmpC/1', 'NotC/1');

    gate_src = {'CmpA/1', 'NotA/1', 'CmpB/1', 'NotB/1', 'CmpC/1', 'NotC/1'};
    for i = 1:6
        add_line(subsys, gate_src{gate_order(i)}, sprintf('Mux_gates/%d', i));
    end
    add_line(subsys, 'Mux_gates/1', 'GatesToDouble/1');
    add_line(subsys, 'GatesToDouble/1', 'gates_6/1');
end

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

function create_foc_controller_subsystem(subsys, ctrl_params, inv_params, motor_params, sim_params)
%CREATE_FOC_CONTROLLER_SUBSYSTEM Fully-wired FOC controller
%   Inputs:  1:ia  2:ib  3:ic  4:theta_e  5:omega_m  6:speed_ref  7:id_ref  8:iq_ref_cmd
%   Outputs: 1:duty_abc[3x1]  2:id_meas  3:iq_meas

    controller_mode = 'pid_sfun';
    if isfield(sim_params, 'controller_mode')
        controller_mode = sim_params.controller_mode;
    end

    if strcmpi(controller_mode, 'sfun')
        create_foc_controller_sfunction(subsys, ctrl_params, inv_params, motor_params, sim_params);
        return;
    elseif strcmpi(controller_mode, 'pid_sfun')
        create_foc_controller_pid_sfunction(subsys, ctrl_params, inv_params, sim_params);
        return;
    end

    % === Input Ports ===
    add_block('built-in/Inport', [subsys '/ia'],        'Port', '1');
    add_block('built-in/Inport', [subsys '/ib'],        'Port', '2');
    add_block('built-in/Inport', [subsys '/ic'],        'Port', '3');
    add_block('built-in/Inport', [subsys '/theta_e'],   'Port', '4');
    add_block('built-in/Inport', [subsys '/omega_m'],   'Port', '5');
    add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
    add_block('built-in/Inport', [subsys '/id_ref'],    'Port', '7');
    add_block('built-in/Inport', [subsys '/iq_ref_cmd'],'Port', '8');

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

function create_foc_controller_sfunction(subsys, ctrl_params, inv_params, motor_params, sim_params)
%CREATE_FOC_CONTROLLER_SFUNCTION C++ S-Function based FOC controller wrapper
%   Inputs:  1:ia  2:ib  3:ic  4:theta_e  5:omega_m  6:speed_ref  7:id_ref  8:iq_ref_cmd
%   Outputs: 1:duty_abc[3x1]  2:id_meas  3:iq_meas

    add_block('built-in/Inport', [subsys '/ia'],        'Port', '1');
    add_block('built-in/Inport', [subsys '/ib'],        'Port', '2');
    add_block('built-in/Inport', [subsys '/ic'],        'Port', '3');
    add_block('built-in/Inport', [subsys '/theta_e'],   'Port', '4');
    add_block('built-in/Inport', [subsys '/omega_m'],   'Port', '5');
    add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
    add_block('built-in/Inport', [subsys '/id_ref'],    'Port', '7');
    add_block('built-in/Inport', [subsys '/iq_ref_cmd'],'Port', '8');

    add_block('built-in/Outport', [subsys '/duty_abc'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/id_meas'],  'Port', '2');
    add_block('built-in/Outport', [subsys '/iq_meas'],  'Port', '3');

    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_in'], ...
        'Inputs', '8', 'Position', [120 90 130 270]);

    Ts = sim_params.Ts_control;
    use_external_iq_ref = 0;
    if isfield(sim_params, 'iq_ref_source') && strcmpi(sim_params.iq_ref_source, 'throttle')
        use_external_iq_ref = 1;
    end

    % Parameters for sfun_foc_controller, in order:
    % [Ts, Vdc, Kp_id, Ki_id, Kp_iq, Ki_iq, Kp_speed, Ki_speed, ...
    %  Rs, Ld, Lq, flux_pm, p, iq_max, id_max, use_external_iq_ref]
    param_values = [ ...
        Ts, ...
        inv_params.Vdc, ...
        ctrl_params.Kp_id, ctrl_params.Ki_id, ...
        ctrl_params.Kp_iq, ctrl_params.Ki_iq, ...
        ctrl_params.Kp_speed, ctrl_params.Ki_speed, ...
        motor_params.Rs, motor_params.Ld, motor_params.Lq, motor_params.flux_pm, ...
        motor_params.p, ctrl_params.iq_max, ctrl_params.id_max, ...
        use_external_iq_ref ...
    ];

    % Generate a robust format string matching the number of parameters
    fmt = repmat('%.12g,', 1, numel(param_values));
    fmt(end) = [];  % remove trailing comma

    params = sprintf(fmt, param_values);
    add_block('simulink/User-Defined Functions/S-Function', [subsys '/FOC_SFun'], ...
        'FunctionName', 'sfun_foc_controller', ...
        'Parameters', params, ...
        'Position', [220 120 380 210]);

    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_out'], ...
        'Outputs', '5', 'Position', [440 120 450 220]);
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_duty'], ...
        'Inputs', '3', 'Position', [500 120 505 190]);

    add_line(subsys, 'ia/1', 'Mux_in/1');
    add_line(subsys, 'ib/1', 'Mux_in/2');
    add_line(subsys, 'ic/1', 'Mux_in/3');
    add_line(subsys, 'theta_e/1', 'Mux_in/4');
    add_line(subsys, 'omega_m/1', 'Mux_in/5');
    add_line(subsys, 'speed_ref/1', 'Mux_in/6');
    add_line(subsys, 'id_ref/1', 'Mux_in/7');
    add_line(subsys, 'iq_ref_cmd/1', 'Mux_in/8');

    add_line(subsys, 'Mux_in/1', 'FOC_SFun/1');
    add_line(subsys, 'FOC_SFun/1', 'Demux_out/1');

    add_line(subsys, 'Demux_out/1', 'Mux_duty/1');
    add_line(subsys, 'Demux_out/2', 'Mux_duty/2');
    add_line(subsys, 'Demux_out/3', 'Mux_duty/3');
    add_line(subsys, 'Mux_duty/1', 'duty_abc/1');
    add_line(subsys, 'Demux_out/4', 'id_meas/1');
    add_line(subsys, 'Demux_out/5', 'iq_meas/1');
end

function create_foc_controller_pid_sfunction(subsys, ctrl_params, inv_params, sim_params)
%CREATE_FOC_CONTROLLER_PID_SFUNCTION FOC with C++ PI S-Function loops.
%   Inputs:  1:ia  2:ib  3:ic  4:theta_e  5:omega_m  6:speed_ref  7:id_ref  8:iq_ref_cmd
%   Outputs: 1:duty_abc[3x1]  2:id_meas  3:iq_meas

    add_block('built-in/Inport', [subsys '/ia'],        'Port', '1');
    add_block('built-in/Inport', [subsys '/ib'],        'Port', '2');
    add_block('built-in/Inport', [subsys '/ic'],        'Port', '3');
    add_block('built-in/Inport', [subsys '/theta_e'],   'Port', '4');
    add_block('built-in/Inport', [subsys '/omega_m'],   'Port', '5');
    add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
    add_block('built-in/Inport', [subsys '/id_ref'],    'Port', '7');
    add_block('built-in/Inport', [subsys '/iq_ref_cmd'],'Port', '8');

    add_block('built-in/Outport', [subsys '/duty_abc'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/id_meas'],  'Port', '2');
    add_block('built-in/Outport', [subsys '/iq_meas'],  'Port', '3');

    % Clarke transform implemented explicitly to avoid vector-width inference issues.
    add_block('simulink/Math Operations/Gain', [subsys '/Ia_gain'], ...
        'Gain', '2/3', 'Position', [120 60 165 85]);
    add_block('simulink/Math Operations/Gain', [subsys '/Ib_alpha_gain'], ...
        'Gain', '-1/3', 'Position', [120 95 165 120]);
    add_block('simulink/Math Operations/Gain', [subsys '/Ic_alpha_gain'], ...
        'Gain', '-1/3', 'Position', [120 130 165 155]);
    add_block('simulink/Math Operations/Sum', [subsys '/Ialpha_calc'], ...
        'Inputs', '+++', 'Position', [195 85 225 125]);

    add_block('simulink/Math Operations/Gain', [subsys '/Ib_beta_gain'], ...
        'Gain', 'sqrt(3)/3', 'Position', [120 180 165 205]);
    add_block('simulink/Math Operations/Gain', [subsys '/Ic_beta_gain'], ...
        'Gain', '-sqrt(3)/3', 'Position', [120 215 165 240]);
    add_block('simulink/Math Operations/Sum', [subsys '/Ibeta_calc'], ...
        'Inputs', '++', 'Position', [195 190 225 225]);

    add_line(subsys, 'ia/1', 'Ia_gain/1');
    add_line(subsys, 'ib/1', 'Ib_alpha_gain/1');
    add_line(subsys, 'ic/1', 'Ic_alpha_gain/1');
    add_line(subsys, 'Ia_gain/1', 'Ialpha_calc/1');
    add_line(subsys, 'Ib_alpha_gain/1', 'Ialpha_calc/2');
    add_line(subsys, 'Ic_alpha_gain/1', 'Ialpha_calc/3');

    add_line(subsys, 'ib/1', 'Ib_beta_gain/1');
    add_line(subsys, 'ic/1', 'Ic_beta_gain/1');
    add_line(subsys, 'Ib_beta_gain/1', 'Ibeta_calc/1');
    add_line(subsys, 'Ic_beta_gain/1', 'Ibeta_calc/2');

    % Park transform implemented explicitly to avoid vector-width inference issues.
    add_block('simulink/Math Operations/Trigonometric Function', [subsys '/Cos_theta'], ...
        'Operator', 'cos', 'Position', [380 70 420 90]);
    add_block('simulink/Math Operations/Trigonometric Function', [subsys '/Sin_theta'], ...
        'Operator', 'sin', 'Position', [380 115 420 135]);

    add_block('simulink/Math Operations/Product', [subsys '/Id_p1'], ...
        'Inputs', '**', 'Position', [450 55 485 85]);
    add_block('simulink/Math Operations/Product', [subsys '/Id_p2'], ...
        'Inputs', '**', 'Position', [450 100 485 130]);
    add_block('simulink/Math Operations/Sum', [subsys '/Id_calc'], ...
        'Inputs', '++', 'Position', [515 75 545 110]);

    add_block('simulink/Math Operations/Product', [subsys '/Iq_p1'], ...
        'Inputs', '**', 'Position', [450 150 485 180]);
    add_block('simulink/Math Operations/Product', [subsys '/Iq_p2'], ...
        'Inputs', '**', 'Position', [450 195 485 225]);
    add_block('simulink/Math Operations/Sum', [subsys '/Iq_calc'], ...
        'Inputs', '-+', 'Position', [515 170 545 205]);

    add_line(subsys, 'theta_e/1', 'Cos_theta/1');
    add_line(subsys, 'theta_e/1', 'Sin_theta/1');

    add_line(subsys, 'Ialpha_calc/1', 'Id_p1/1');
    add_line(subsys, 'Cos_theta/1', 'Id_p1/2');
    add_line(subsys, 'Ibeta_calc/1', 'Id_p2/1');
    add_line(subsys, 'Sin_theta/1', 'Id_p2/2');
    add_line(subsys, 'Id_p1/1', 'Id_calc/1');
    add_line(subsys, 'Id_p2/1', 'Id_calc/2');

    add_line(subsys, 'Ialpha_calc/1', 'Iq_p1/1');
    add_line(subsys, 'Sin_theta/1', 'Iq_p1/2');
    add_line(subsys, 'Ibeta_calc/1', 'Iq_p2/1');
    add_line(subsys, 'Cos_theta/1', 'Iq_p2/2');
    add_line(subsys, 'Iq_p1/1', 'Iq_calc/1');
    add_line(subsys, 'Iq_p2/1', 'Iq_calc/2');

    add_line(subsys, 'Id_calc/1', 'id_meas/1');
    add_line(subsys, 'Iq_calc/1', 'iq_meas/1');

    % Speed PI via C++ S-Function: iq_ref = PI(speed_ref_rad_s - omega_m)
    add_block('simulink/Math Operations/Gain', [subsys '/RPM2RadS'], ...
        'Gain', '2*pi/60', ...
        'Position', [80 200 120 220]);
    add_block('simulink/Math Operations/Sum', [subsys '/Speed Error'], ...
        'Inputs', '+-', 'Position', [160 200 190 230]);

    speed_params = sprintf('%.12g,%.12g,%.12g,%.12g,%.12g', ...
        sim_params.Ts_control, ctrl_params.Kp_speed, ctrl_params.Ki_speed, ...
        -ctrl_params.iq_max, ctrl_params.iq_max);
    add_block('simulink/User-Defined Functions/S-Function', [subsys '/Speed PI SFun'], ...
        'FunctionName', 'sfun_pi_controller', ...
        'Parameters', speed_params, ...
        'Position', [230 200 360 240]);

    iq_ref_source = 'speed_pi';
    if isfield(sim_params, 'iq_ref_source') && ~isempty(sim_params.iq_ref_source)
        iq_ref_source = lower(sim_params.iq_ref_source);
    end
    add_block('simulink/Signal Routing/Switch', [subsys '/IqRef Select'], ...
        'Threshold', '0.5', ...
        'Criteria', 'u2 >= Threshold', ...
        'Position', [420 215 455 255]);
    add_block('built-in/Constant', [subsys '/UseExternalIqRef'], ...
        'Value', num2str(double(strcmp(iq_ref_source, 'throttle'))), ...
        'Position', [360 255 405 275]);

    add_line(subsys, 'speed_ref/1', 'RPM2RadS/1');
    add_line(subsys, 'RPM2RadS/1', 'Speed Error/1');
    add_line(subsys, 'omega_m/1', 'Speed Error/2');
    add_line(subsys, 'Speed Error/1', 'Speed PI SFun/1');
    add_line(subsys, 'Speed PI SFun/1', 'IqRef Select/1');
    add_line(subsys, 'UseExternalIqRef/1', 'IqRef Select/2');
    add_line(subsys, 'iq_ref_cmd/1', 'IqRef Select/3');

    % Current PI via C++ S-Function
    add_block('simulink/Math Operations/Sum', [subsys '/Id Error'], ...
        'Inputs', '+-', 'Position', [600 160 630 190]);
    add_block('simulink/Math Operations/Sum', [subsys '/Iq Error'], ...
        'Inputs', '+-', 'Position', [600 250 630 280]);

    v_limit = inv_params.Vdc / sqrt(3);
    id_params = sprintf('%.12g,%.12g,%.12g,%.12g,%.12g', ...
        sim_params.Ts_control, ctrl_params.Kp_id, ctrl_params.Ki_id, ...
        -v_limit, v_limit);
    iq_params = sprintf('%.12g,%.12g,%.12g,%.12g,%.12g', ...
        sim_params.Ts_control, ctrl_params.Kp_iq, ctrl_params.Ki_iq, ...
        -v_limit, v_limit);

    add_block('simulink/User-Defined Functions/S-Function', [subsys '/Id PI SFun'], ...
        'FunctionName', 'sfun_pi_controller', ...
        'Parameters', id_params, ...
        'Position', [670 160 800 200]);
    add_block('simulink/User-Defined Functions/S-Function', [subsys '/Iq PI SFun'], ...
        'FunctionName', 'sfun_pi_controller', ...
        'Parameters', iq_params, ...
        'Position', [670 250 800 290]);

    add_line(subsys, 'id_ref/1', 'Id Error/1');
    add_line(subsys, 'Id_calc/1', 'Id Error/2');
    add_line(subsys, 'Id Error/1', 'Id PI SFun/1');

    add_line(subsys, 'IqRef Select/1', 'Iq Error/1');
    add_line(subsys, 'Iq_calc/1', 'Iq Error/2');
    add_line(subsys, 'Iq Error/1', 'Iq PI SFun/1');

    % Inverse Park: [vd, vq, theta_e] -> [v_alpha, v_beta]
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_invpark'], ...
        'Inputs', '3', 'Position', [840 180 845 280]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/InvPark'], ...
        'MATLABFcn', '[u(1)*cos(u(3))-u(2)*sin(u(3)); u(1)*sin(u(3))+u(2)*cos(u(3))]', ...
        'Position', [880 210 1000 250]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_vab'], ...
        'Outputs', '2', 'Position', [1030 210 1035 250]);

    add_line(subsys, 'Id PI SFun/1', 'Mux_invpark/1');
    add_line(subsys, 'Iq PI SFun/1', 'Mux_invpark/2');
    add_line(subsys, 'theta_e/1', 'Mux_invpark/3');
    add_line(subsys, 'Mux_invpark/1', 'InvPark/1');
    add_line(subsys, 'InvPark/1', 'Demux_vab/1');

    % SVPWM: [v_alpha, v_beta, Vdc] -> [da; db; dc]
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_svpwm'], ...
        'Inputs', '3', 'Position', [1070 200 1075 270]);
    add_block('built-in/Constant', [subsys '/Vdc_const'], ...
        'Value', num2str(inv_params.Vdc), ...
        'Position', [1020 270 1060 290]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/SVPWM'], ...
        'MATLABFcn', 'svpwm_inline(u(1), u(2), u(3))', ...
        'Position', [1110 210 1220 250]);

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

function create_measurement_subsystem(subsys, motor_params, sim_params)
%CREATE_MEASUREMENT_SUBSYSTEM Pass-through currents + theta_e + omega_m
%   Inputs:  1:ia  2:ib  3:ic  4:theta_m  5:omega_m
%   Outputs: 1:ia_meas  2:ib_meas  3:ic_meas  4:theta_e  5:omega_m

    current_noise_std = 0.0;
    theta_noise_std = 0.0;
    noise_seed = 100;
    if isfield(sim_params, 'current_noise_std')
        current_noise_std = sim_params.current_noise_std;
    end
    if isfield(sim_params, 'theta_noise_std')
        theta_noise_std = sim_params.theta_noise_std;
    end
    if isfield(sim_params, 'noise_seed')
        noise_seed = sim_params.noise_seed;
    end
    Ts = sim_params.Ts_control;
    plant_mode = 'code';
    if isfield(sim_params, 'plant_mode')
        plant_mode = sim_params.plant_mode;
    end

    current_meas_lpf_hz = 0.0;
    if isfield(sim_params, 'current_meas_lpf_hz')
        current_meas_lpf_hz = sim_params.current_meas_lpf_hz;
    elseif strcmpi(plant_mode, 'simscape_blue')
        current_meas_lpf_hz = 2000.0;
    end

    omega_meas_lpf_hz = 0.0;
    if isfield(sim_params, 'omega_meas_lpf_hz')
        omega_meas_lpf_hz = sim_params.omega_meas_lpf_hz;
    elseif strcmpi(plant_mode, 'simscape_blue')
        omega_meas_lpf_hz = 500.0;
    end

    theta_e_mode = 'p_times_theta_m';
    if isfield(sim_params, 'theta_e_mode') && ~isempty(sim_params.theta_e_mode)
        theta_e_mode = sim_params.theta_e_mode;
    end
    theta_e_offset = 0.0;
    if isfield(sim_params, 'theta_e_offset')
        theta_e_offset = sim_params.theta_e_offset;
    end

    switch lower(theta_e_mode)
        case 'p_times_theta_m'
            theta_gain = motor_params.p;
        case 'theta_m'
            theta_gain = 1.0;
        case 'neg_p_times_theta_m'
            theta_gain = -motor_params.p;
        case 'neg_theta_m'
            theta_gain = -1.0;
        otherwise
            error('Unsupported theta_e_mode: %s', theta_e_mode);
    end

    add_block('built-in/Inport',  [subsys '/ia'],        'Port', '1');
    add_block('built-in/Inport',  [subsys '/ib'],        'Port', '2');
    add_block('built-in/Inport',  [subsys '/ic'],        'Port', '3');
    add_block('built-in/Inport',  [subsys '/theta_m'],   'Port', '4');
    add_block('built-in/Inport',  [subsys '/omega_m_in'],'Port', '5');

    add_block('built-in/Outport', [subsys '/ia_meas'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/ib_meas'], 'Port', '2');
    add_block('built-in/Outport', [subsys '/ic_meas'], 'Port', '3');
    add_block('built-in/Outport', [subsys '/theta_e'], 'Port', '4');
    add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '5');

    current_noise_var_str = num2str(current_noise_std^2, '%.12g');
    add_block('simulink/Sources/Random Number', [subsys '/Noise_ia'], ...
        'Mean', '0', ...
        'Variance', current_noise_var_str, ...
        'Seed', num2str(noise_seed + 1), ...
        'SampleTime', num2str(Ts), ...
        'Position', [120 20 170 40]);
    add_block('simulink/Sources/Random Number', [subsys '/Noise_ib'], ...
        'Mean', '0', ...
        'Variance', current_noise_var_str, ...
        'Seed', num2str(noise_seed + 2), ...
        'SampleTime', num2str(Ts), ...
        'Position', [120 70 170 90]);
    add_block('simulink/Sources/Random Number', [subsys '/Noise_ic'], ...
        'Mean', '0', ...
        'Variance', current_noise_var_str, ...
        'Seed', num2str(noise_seed + 3), ...
        'SampleTime', num2str(Ts), ...
        'Position', [120 120 170 140]);

    add_block('simulink/Math Operations/Sum', [subsys '/Sum_ia'], ...
        'Inputs', '++', 'Position', [220 15 250 45]);
    add_block('simulink/Math Operations/Sum', [subsys '/Sum_ib'], ...
        'Inputs', '++', 'Position', [220 65 250 95]);
    add_block('simulink/Math Operations/Sum', [subsys '/Sum_ic'], ...
        'Inputs', '++', 'Position', [220 115 250 145]);

    add_line(subsys, 'ia/1', 'Sum_ia/1');
    add_line(subsys, 'Noise_ia/1', 'Sum_ia/2');

    add_line(subsys, 'ib/1', 'Sum_ib/1');
    add_line(subsys, 'Noise_ib/1', 'Sum_ib/2');

    add_line(subsys, 'ic/1', 'Sum_ic/1');
    add_line(subsys, 'Noise_ic/1', 'Sum_ic/2');

    % theta_e = pole_pairs * theta_m (+ optional measurement noise)
    add_block('simulink/Math Operations/Gain', [subsys '/PoleP'], ...
        'Gain', num2str(theta_gain, '%.12g'), ...
        'Position', [200 180 250 210]);
    add_block('simulink/Sources/Random Number', [subsys '/Noise_theta'], ...
        'Mean', '0', ...
        'Variance', num2str(theta_noise_std^2, '%.12g'), ...
        'Seed', num2str(noise_seed + 4), ...
        'SampleTime', num2str(Ts), ...
        'Position', [280 190 330 210]);
    add_block('simulink/Math Operations/Sum', [subsys '/Sum_theta'], ...
        'Inputs', '++', ...
        'Position', [360 180 390 210]);
    add_block('simulink/Math Operations/Bias', [subsys '/ThetaOffset'], ...
        'Bias', num2str(theta_e_offset, '%.12g'), ...
        'Position', [410 180 450 210]);

    add_line(subsys, 'theta_m/1', 'PoleP/1');
    add_line(subsys, 'PoleP/1', 'Sum_theta/1');
    add_line(subsys, 'Noise_theta/1', 'Sum_theta/2');
    add_line(subsys, 'Sum_theta/1', 'ThetaOffset/1');

    if strcmpi(plant_mode, 'simscape_blue')
        add_block('simulink/Discrete/Zero-Order Hold', [subsys '/ZOH_ia'], ...
            'SampleTime', num2str(Ts), 'Position', [300 15 350 45]);
        add_block('simulink/Discrete/Zero-Order Hold', [subsys '/ZOH_ib'], ...
            'SampleTime', num2str(Ts), 'Position', [300 65 350 95]);
        add_block('simulink/Discrete/Zero-Order Hold', [subsys '/ZOH_ic'], ...
            'SampleTime', num2str(Ts), 'Position', [300 115 350 145]);
        add_block('simulink/Discrete/Zero-Order Hold', [subsys '/ZOH_theta'], ...
            'SampleTime', num2str(Ts), 'Position', [420 180 470 210]);
        add_block('simulink/Discrete/Zero-Order Hold', [subsys '/ZOH_omega'], ...
            'SampleTime', num2str(Ts), 'Position', [300 250 350 280]);

        add_line(subsys, 'Sum_ia/1', 'ZOH_ia/1');
        add_line(subsys, 'Sum_ib/1', 'ZOH_ib/1');
        add_line(subsys, 'Sum_ic/1', 'ZOH_ic/1');
        add_line(subsys, 'ThetaOffset/1', 'ZOH_theta/1');
        add_line(subsys, 'omega_m_in/1', 'ZOH_omega/1');

        current_alpha = compute_lpf_alpha(current_meas_lpf_hz, Ts);
        omega_alpha = compute_lpf_alpha(omega_meas_lpf_hz, Ts);

        if current_alpha > 0
            create_first_order_discrete_lpf(subsys, 'LPF_ia', current_alpha, [390 15 470 45]);
            create_first_order_discrete_lpf(subsys, 'LPF_ib', current_alpha, [390 65 470 95]);
            create_first_order_discrete_lpf(subsys, 'LPF_ic', current_alpha, [390 115 470 145]);
            add_line(subsys, 'ZOH_ia/1', 'LPF_ia/1');
            add_line(subsys, 'ZOH_ib/1', 'LPF_ib/1');
            add_line(subsys, 'ZOH_ic/1', 'LPF_ic/1');
            add_line(subsys, 'LPF_ia/1', 'ia_meas/1');
            add_line(subsys, 'LPF_ib/1', 'ib_meas/1');
            add_line(subsys, 'LPF_ic/1', 'ic_meas/1');
        else
            add_line(subsys, 'ZOH_ia/1', 'ia_meas/1');
            add_line(subsys, 'ZOH_ib/1', 'ib_meas/1');
            add_line(subsys, 'ZOH_ic/1', 'ic_meas/1');
        end

        add_line(subsys, 'ZOH_theta/1', 'theta_e/1');

        if omega_alpha > 0
            create_first_order_discrete_lpf(subsys, 'LPF_omega', omega_alpha, [390 250 470 280]);
            add_line(subsys, 'ZOH_omega/1', 'LPF_omega/1');
            add_line(subsys, 'LPF_omega/1', 'omega_m/1');
        else
            add_line(subsys, 'ZOH_omega/1', 'omega_m/1');
        end
    else
        add_line(subsys, 'Sum_ia/1', 'ia_meas/1');
        add_line(subsys, 'Sum_ib/1', 'ib_meas/1');
        add_line(subsys, 'Sum_ic/1', 'ic_meas/1');
        add_line(subsys, 'ThetaOffset/1', 'theta_e/1');

        % omega_m: direct pass-through (no Derivative block to avoid noise amplification)
        add_line(subsys, 'omega_m_in/1', 'omega_m/1');
    end
end

function alpha = compute_lpf_alpha(cutoff_hz, Ts)
%COMPUTE_LPF_ALPHA Convert cutoff frequency to discrete first-order filter alpha.
    if cutoff_hz <= 0
        alpha = 0.0;
        return;
    end

    tau = 1 / (2 * pi * cutoff_hz);
    alpha = exp(-Ts / tau);
end

function create_first_order_discrete_lpf(subsys, name, alpha, position)
%CREATE_FIRST_ORDER_DISCRETE_LPF y[k] = (1-alpha)u[k] + alpha y[k-1]
    numerator = sprintf('[%.12g]', 1 - alpha);
    denominator = sprintf('[1 %.12g]', -alpha);
    add_block('simulink/Discrete/Discrete Transfer Fcn', [subsys '/' name], ...
        'Numerator', numerator, ...
        'Denominator', denominator, ...
        'InitialStates', '0', ...
        'Position', position);
end

function create_signals_subsystem(subsys, ref_params, motor_params, ctrl_params, sim_params)
%CREATE_SIGNALS_SUBSYSTEM Unified system input module (reference + load).
% Outputs:
%   1:speed_ref_rpm 2:id_ref 3:iq_ref_cmd 4:Te_load

    add_block('built-in/Outport', [subsys '/speed_ref'], 'Port', '1');
    add_block('built-in/Outport', [subsys '/id_ref'], 'Port', '2');
    add_block('built-in/Outport', [subsys '/iq_ref_cmd'], 'Port', '3');
    add_block('built-in/Outport', [subsys '/Te_load'], 'Port', '4');

    if ref_params.speed_ramp_time <= 0
        speed_slope = 0;
    else
        speed_slope = ref_params.speed_ref / ref_params.speed_ramp_time;
    end

    add_block('simulink/Sources/Ramp', [subsys '/Speed Ramp'], ...
        'Slope', num2str(speed_slope), ...
        'Position', [45 25 115 55]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Speed Sat'], ...
        'UpperLimit', num2str(ref_params.speed_ref), ...
        'LowerLimit', '0', ...
        'Position', [145 25 205 55]);
    add_block('built-in/Constant', [subsys '/Id Ref'], ...
        'Value', num2str(ref_params.id_ref), ...
        'Position', [60 80 110 100]);

    add_line(subsys, 'Speed Ramp/1', 'Speed Sat/1');
    add_line(subsys, 'Speed Sat/1', 'speed_ref/1');
    add_line(subsys, 'Id Ref/1', 'id_ref/1');

    throttle_adc_before = 0;
    if isfield(ref_params, 'throttle_adc_before')
        throttle_adc_before = ref_params.throttle_adc_before;
    end
    throttle_adc_after = 2048;
    if isfield(ref_params, 'throttle_adc_after')
        throttle_adc_after = ref_params.throttle_adc_after;
    end
    throttle_step_time = 0.05;
    if isfield(ref_params, 'throttle_step_time')
        throttle_step_time = ref_params.throttle_step_time;
    end
    throttle_torque_max = 0.2;
    if isfield(ref_params, 'throttle_torque_max')
        throttle_torque_max = ref_params.throttle_torque_max;
    end
    throttle_ts = 1e-3;
    if isfield(sim_params, 'Ts_throttle')
        throttle_ts = sim_params.Ts_throttle;
    end

    torque_to_iq = 1 / (1.5 * motor_params.p * motor_params.flux_pm);

    add_block('simulink/Sources/Step', [subsys '/Throttle ADC Step'], ...
        'Time', num2str(throttle_step_time), ...
        'Before', num2str(throttle_adc_before), ...
        'After', num2str(throttle_adc_after), ...
        'Position', [45 130 125 160]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/ADC Sat'], ...
        'LowerLimit', '0', ...
        'UpperLimit', '4095', ...
        'Position', [150 130 210 160]);
    add_block('simulink/Discrete/Zero-Order Hold', [subsys '/Throttle ZOH'], ...
        'SampleTime', num2str(throttle_ts), ...
        'Position', [235 130 295 160]);
    add_block('simulink/Math Operations/Gain', [subsys '/ADC To Norm'], ...
        'Gain', '1/4095', ...
        'Position', [320 130 380 160]);
    add_block('simulink/Math Operations/Gain', [subsys '/Norm To Torque'], ...
        'Gain', num2str(throttle_torque_max), ...
        'Position', [405 130 485 160]);
    add_block('simulink/Math Operations/Gain', [subsys '/Torque To Iq'], ...
        'Gain', num2str(torque_to_iq, '%.12g'), ...
        'Position', [510 130 590 160]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Iq Ref Sat'], ...
        'UpperLimit', num2str(ctrl_params.iq_max), ...
        'LowerLimit', num2str(-ctrl_params.iq_max), ...
        'Position', [615 130 685 160]);

    add_line(subsys, 'Throttle ADC Step/1', 'ADC Sat/1');
    add_line(subsys, 'ADC Sat/1', 'Throttle ZOH/1');
    add_line(subsys, 'Throttle ZOH/1', 'ADC To Norm/1');
    add_line(subsys, 'ADC To Norm/1', 'Norm To Torque/1');
    add_line(subsys, 'Norm To Torque/1', 'Torque To Iq/1');
    add_line(subsys, 'Torque To Iq/1', 'Iq Ref Sat/1');
    add_line(subsys, 'Iq Ref Sat/1', 'iq_ref_cmd/1');

    add_block('simulink/Sources/Step', [subsys '/Load Step'], ...
        'Time', num2str(ref_params.load_step_time), ...
        'Before', '0', ...
        'After', num2str(ref_params.load_torque), ...
        'Position', [45 185 125 215]);
    add_line(subsys, 'Load Step/1', 'Te_load/1');

end

function wire_scope_compare_paths(model_name, plant_mode)
%WIRE_SCOPE_COMPARE_PATHS Build direct branch compare paths before each scope.

    add_block('simulink/Math Operations/Gain', [model_name '/SpeedRefRadS'], ...
        'Gain', '2*pi/60', 'Position', [930 60 995 80]);
    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_speed_scope'], ...
        'Inputs', '2', 'Position', [1065 55 1070 105]);

    add_line(model_name, 'signals/1', 'SpeedRefRadS/1', 'autorouting', 'smart');
    add_line(model_name, 'SpeedRefRadS/1', 'Mux_speed_scope/2', 'autorouting', 'smart');
    if strcmpi(plant_mode, 'simscape_blue')
        add_line(model_name, 'Simscape Plant/4', 'Mux_speed_scope/1', 'autorouting', 'smart');
    else
        add_line(model_name, 'PMSM Motor/4', 'Mux_speed_scope/1', 'autorouting', 'smart');
    end
    add_line(model_name, 'Mux_speed_scope/1', 'Speed Scope/1', 'autorouting', 'smart');

    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_current_scope'], ...
        'Inputs', '4', 'Position', [1035 165 1040 285]);

    add_line(model_name, 'signals/2', 'Mux_current_scope/1', 'autorouting', 'smart');
    add_line(model_name, 'FOC Controller/2', 'Mux_current_scope/2', 'autorouting', 'smart');
    add_line(model_name, 'signals/3', 'Mux_current_scope/3', 'autorouting', 'smart');
    add_line(model_name, 'FOC Controller/3', 'Mux_current_scope/4', 'autorouting', 'smart');
    add_line(model_name, 'Mux_current_scope/1', 'Current Scope/1', 'autorouting', 'smart');

    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_torque_scope'], ...
        'Inputs', '2', 'Position', [1065 315 1070 365]);

    add_line(model_name, 'signals/4', 'Mux_torque_scope/1', 'autorouting', 'smart');
    if strcmpi(plant_mode, 'simscape_blue')
        add_line(model_name, 'Simscape Plant/5', 'Mux_torque_scope/2', 'autorouting', 'smart');
    else
        add_line(model_name, 'PMSM Motor/5', 'Mux_torque_scope/2', 'autorouting', 'smart');
    end
    add_line(model_name, 'Mux_torque_scope/1', 'Torque Scope/1', 'autorouting', 'smart');
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

function create_debug_logging(model_name, plant_mode, plant_input_mode)
%CREATE_DEBUG_LOGGING Add To Workspace taps for controller/measurement diagnostics.

    add_to_workspace_tap(model_name, 'FOC Controller/2', 'id_meas_ts', [870 520 980 550]);
    add_to_workspace_tap(model_name, 'FOC Controller/3', 'iq_meas_ts', [870 555 980 585]);
    add_to_workspace_tap(model_name, 'Measurements/1', 'ia_meas_ts', [760 640 870 670]);
    add_to_workspace_tap(model_name, 'Measurements/2', 'ib_meas_ts', [760 675 870 705]);
    add_to_workspace_tap(model_name, 'Measurements/3', 'ic_meas_ts', [760 710 870 740]);
    add_to_workspace_tap(model_name, 'Measurements/4', 'theta_e_ts', [760 745 870 775]);
    add_to_workspace_tap(model_name, 'Measurements/5', 'omega_m_ts', [760 780 870 810]);
    add_to_workspace_tap(model_name, 'FOC Controller/1', 'duty_abc_ts', [870 450 980 480]);

    if strcmpi(plant_mode, 'simscape_blue') && strcmpi(plant_input_mode, 'gates_6')
        add_to_workspace_tap(model_name, 'Gate Driver/1', 'gates_6_ts', [870 415 980 445]);
    end
end

function add_to_workspace_tap(model_name, src_port, var_name, pos)
%ADD_TO_WORKSPACE_TAP Branch a signal to a To Workspace block.
    blk = [model_name '/' var_name '_log'];
    add_block('simulink/Sinks/To Workspace', blk, ...
        'VariableName', var_name, ...
        'SaveFormat', 'Timeseries', ...
        'MaxDataPoints', 'inf', ...
        'Position', pos);
    add_line(model_name, src_port, [var_name '_log/1'], 'autorouting', 'smart');
end
