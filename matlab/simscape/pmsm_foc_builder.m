function varargout = pmsm_foc_builder(action, varargin)
%PMSM_FOC_BUILDER Shared builder for modular PMSM FOC Simulink models.

switch lower(action)
    case 'default_parameters'
        [varargout{1:nargout}] = default_parameters();

    case 'create_module_model'
        [module_name, model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params] = ...
            parse_create_inputs(varargin{:});
        varargout{1} = create_module_model( ...
            module_name, model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

    case 'create_all_in_model'
        [model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params] = ...
            parse_all_in_inputs(varargin{:});
        varargout{1} = create_all_in_model( ...
            model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

    case 'validate_model'
        [model_name, stop_time] = parse_validate_inputs(varargin{:});
        varargout{1} = validate_model(model_name, stop_time);

    case 'derive_pi_ctrl_params'
        if numel(varargin) < 3
            error('derive_pi_ctrl_params requires ctrl_params, motor_params, inv_params.');
        end
        ctrl_params = varargin{1};
        motor_params = varargin{2};
        inv_params = varargin{3};
        varargout{1} = derive_pi_ctrl_params(ctrl_params, motor_params, inv_params);

    otherwise
        error('Unsupported builder action: %s', action);
end
end

function [module_name, model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params] = parse_create_inputs(varargin)
if isempty(varargin)
    error('create_module_model requires at least a module name.');
end

module_name = lower(string(varargin{1}));
if nargin < 2 || isempty(varargin{2})
    model_name = sprintf('pmsm_module_%s', char(module_name));
else
    model_name = char(varargin{2});
end

if numel(varargin) >= 7
    motor_params = varargin{3};
    inv_params = varargin{4};
    ctrl_params = varargin{5};
    sim_params = varargin{6};
    ref_params = varargin{7};
else
    [motor_params, inv_params, ctrl_params, sim_params, ref_params] = default_parameters();
end
end

function [model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params] = parse_all_in_inputs(varargin)
if isempty(varargin)
    model_name = 'pmsm_foc_model';
    [motor_params, inv_params, ctrl_params, sim_params, ref_params] = default_parameters();
    return;
end

model_name = char(varargin{1});
if numel(varargin) >= 6
    motor_params = varargin{2};
    inv_params = varargin{3};
    ctrl_params = varargin{4};
    sim_params = varargin{5};
    ref_params = varargin{6};
else
    [motor_params, inv_params, ctrl_params, sim_params, ref_params] = default_parameters();
end
end

function [model_name, stop_time] = parse_validate_inputs(varargin)
if isempty(varargin)
    error('validate_model requires a model name.');
end

model_name = char(varargin{1});
if numel(varargin) >= 2 && ~isempty(varargin{2})
    stop_time = varargin{2};
else
    stop_time = 0.02;
end
end

function [motor_params, inv_params, ctrl_params, sim_params, ref_params] = default_parameters()
motor_params = struct();
motor_params.Rs = 0.5;
motor_params.Ld = 1.4e-3;
motor_params.Lq = 1.4e-3;
motor_params.flux_pm = 0.0577;
motor_params.p = 4;
motor_params.J = 1.74e-5;
motor_params.B = 1e-4;

inv_params = struct();
inv_params.Vdc = 48;
inv_params.fsw = 20e3;
inv_params.Tsw = 1 / inv_params.fsw;
inv_params.dead_time = 1e-6;

ctrl_params = struct();
ctrl_params.omega_ci = 0.0;
ctrl_params.omega_cs = 0.0;
ctrl_params.Kp_id = 0.0;
ctrl_params.Ki_id = 0.0;
ctrl_params.Kp_iq = 0.0;
ctrl_params.Ki_iq = 0.0;
ctrl_params.Kp_speed = 0.0;
ctrl_params.Ki_speed = 0.0;
ctrl_params.id_max = 10.0;
ctrl_params.iq_max = 10.0;
ctrl_params.speed_max = 3000;
ctrl_params.auto_tune_current = true;
ctrl_params.auto_tune_speed = true;
ctrl_params.enable_internal_speed_loop = false;

sim_params = struct();
sim_params.Ts_control = 1 / inv_params.fsw;
sim_params.Ts_speed = 10 * sim_params.Ts_control;
sim_params.t_end = 0.5;
sim_params.solver = 'ode23t';
sim_params.max_step = 1e-5;
sim_params.validation_t_end = 0.02;

ref_params = struct();
ref_params.speed_ref = 1000;
ref_params.speed_ramp_time = 0.1;
ref_params.load_torque = 0.1;
ref_params.load_step_time = 0.3;
ref_params.id_ref = 0;
ref_params.throttle = 0.0;
ref_params.use_external_inputs = false;
ref_params.throttle_speed_kp = 0.05;
ref_params.throttle_torque_max = 0.8;
ref_params.throttle_iq_max = ctrl_params.iq_max;

ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params, inv_params);
end

function model_path = create_module_model(module_name, model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params, inv_params);
short_stop_time = min(sim_params.validation_t_end, sim_params.t_end);
model_path = initialize_model(model_name, sim_params, short_stop_time);

switch char(module_name)
    case 'signal_in'
        add_block('built-in/Subsystem', [model_name '/Signal In']);
        create_signal_in_subsystem([model_name '/Signal In'], ref_params);
        set_param([model_name '/Signal In'], 'Position', [120 80 300 240]);

        add_ramp_source(model_name, 'CmdSpeedRef', num2str(ref_params.speed_ref / max(ref_params.speed_ramp_time, 1e-6), '%.16g'), [20 70 95 100]);
        add_constant_source(model_name, 'CmdIdRef', num2str(ref_params.id_ref, '%.16g'), [20 120 95 140]);
        add_step_source(model_name, 'CmdLoad', num2str(ref_params.load_step_time, '%.16g'), '0', num2str(ref_params.load_torque, '%.16g'), [10 165 95 195]);
        add_constant_source(model_name, 'CmdThrottle', num2str(ref_params.throttle, '%.16g'), [20 215 95 235]);

        add_scope_block([model_name '/Speed Ref Scope'], [430 80 500 120]);
        add_scope_block([model_name '/Id Ref Scope'], [430 145 500 185]);
        add_scope_block([model_name '/Load Scope'], [430 210 500 250]);
        add_scope_block([model_name '/Throttle Scope'], [430 275 520 315]);

        add_line(model_name, 'CmdSpeedRef/1', 'Signal In/1', 'autorouting', 'smart');
        add_line(model_name, 'CmdIdRef/1', 'Signal In/2', 'autorouting', 'smart');
        add_line(model_name, 'CmdLoad/1', 'Signal In/3', 'autorouting', 'smart');
        add_line(model_name, 'CmdThrottle/1', 'Signal In/4', 'autorouting', 'smart');

        add_line(model_name, 'Signal In/1', 'Speed Ref Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Signal In/2', 'Id Ref Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Signal In/3', 'Load Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Signal In/4', 'Throttle Scope/1', 'autorouting', 'smart');

    case 'thro'
        add_block('built-in/Subsystem', [model_name '/Thro']);
        create_thro_subsystem([model_name '/Thro'], ref_params, ctrl_params);
        set_param([model_name '/Thro'], 'Position', [250 110 500 310]);

        add_constant_source(model_name, 'CurrentSpeed', '0.0', [40 120 105 140]);
        add_step_source(model_name, 'TargetSpeed', '0.001', '0', num2str(ref_params.speed_ref), [25 175 105 205]);
        add_constant_source(model_name, 'Throttle', num2str(ref_params.throttle), [40 235 105 255]);

        add_scope_block([model_name '/Iq Ref Scope'], [590 175 680 215]);

        add_line(model_name, 'CurrentSpeed/1', 'Thro/1', 'autorouting', 'smart');
        add_line(model_name, 'TargetSpeed/1', 'Thro/2', 'autorouting', 'smart');
        add_line(model_name, 'Throttle/1', 'Thro/3', 'autorouting', 'smart');
        add_line(model_name, 'Thro/1', 'Iq Ref Scope/1', 'autorouting', 'smart');

    case 'foc_controller'
        add_block('built-in/Subsystem', [model_name '/FOC Controller']);
        create_foc_controller_subsystem([model_name '/FOC Controller'], ctrl_params, inv_params, motor_params, sim_params);
        set_param([model_name '/FOC Controller'], 'Position', [280 80 560 390]);

        add_constant_source(model_name, 'Ia', '0.0', [40 60 90 80]);
        add_constant_source(model_name, 'Ib', '-1.0', [40 105 90 125]);
        add_constant_source(model_name, 'Ic', '1.0', [40 150 90 170]);
        add_ramp_source(model_name, 'ThetaE', '2*pi*50', [30 195 100 225]);
        add_constant_source(model_name, 'OmegaM', '0.0', [40 245 90 265]);
        add_step_source(model_name, 'SpeedRef', '0.001', '0', num2str(ref_params.speed_ref), [25 290 100 320]);
        add_constant_source(model_name, 'IdRef', num2str(ref_params.id_ref), [40 345 90 365]);
        add_constant_source(model_name, 'IqRef', '0.0', [40 390 100 410]);
        add_constant_source(model_name, 'KpSpeedRt', num2str(ctrl_params.Kp_speed, '%.16g'), [25 430 105 450]);
        add_constant_source(model_name, 'KiSpeedRt', num2str(ctrl_params.Ki_speed, '%.16g'), [25 470 105 490]);
        add_constant_source(model_name, 'IqMaxSpeedRt', num2str(ctrl_params.iq_max, '%.16g'), [15 510 110 530]);
        add_constant_source(model_name, 'KpIdRt', num2str(ctrl_params.Kp_id, '%.16g'), [140 430 220 450]);
        add_constant_source(model_name, 'KiIdRt', num2str(ctrl_params.Ki_id, '%.16g'), [140 470 220 490]);
        add_constant_source(model_name, 'KpIqRt', num2str(ctrl_params.Kp_iq, '%.16g'), [140 510 220 530]);
        add_constant_source(model_name, 'KiIqRt', num2str(ctrl_params.Ki_iq, '%.16g'), [140 550 220 570]);
        add_constant_source(model_name, 'IdMaxRt', num2str(ctrl_params.id_max, '%.16g'), [240 470 320 490]);
        add_constant_source(model_name, 'IqMaxRt', num2str(ctrl_params.iq_max, '%.16g'), [240 510 320 530]);

        add_scope_block([model_name '/Duty Scope'], [650 110 720 150]);
        add_scope_block([model_name '/Id Scope'], [650 190 720 230]);
        add_scope_block([model_name '/Iq Scope'], [650 270 720 310]);

        add_line(model_name, 'Ia/1', 'FOC Controller/1', 'autorouting', 'smart');
        add_line(model_name, 'Ib/1', 'FOC Controller/2', 'autorouting', 'smart');
        add_line(model_name, 'Ic/1', 'FOC Controller/3', 'autorouting', 'smart');
        add_line(model_name, 'ThetaE/1', 'FOC Controller/4', 'autorouting', 'smart');
        add_line(model_name, 'OmegaM/1', 'FOC Controller/5', 'autorouting', 'smart');
        add_line(model_name, 'SpeedRef/1', 'FOC Controller/6', 'autorouting', 'smart');
        add_line(model_name, 'IdRef/1', 'FOC Controller/7', 'autorouting', 'smart');
        add_line(model_name, 'IqRef/1', 'FOC Controller/8', 'autorouting', 'smart');
        add_line(model_name, 'KpSpeedRt/1', 'FOC Controller/9', 'autorouting', 'smart');
        add_line(model_name, 'KiSpeedRt/1', 'FOC Controller/10', 'autorouting', 'smart');
        add_line(model_name, 'IqMaxSpeedRt/1', 'FOC Controller/11', 'autorouting', 'smart');
        add_line(model_name, 'KpIdRt/1', 'FOC Controller/12', 'autorouting', 'smart');
        add_line(model_name, 'KiIdRt/1', 'FOC Controller/13', 'autorouting', 'smart');
        add_line(model_name, 'KpIqRt/1', 'FOC Controller/14', 'autorouting', 'smart');
        add_line(model_name, 'KiIqRt/1', 'FOC Controller/15', 'autorouting', 'smart');
        add_line(model_name, 'IdMaxRt/1', 'FOC Controller/16', 'autorouting', 'smart');
        add_line(model_name, 'IqMaxRt/1', 'FOC Controller/17', 'autorouting', 'smart');
        add_line(model_name, 'FOC Controller/1', 'Duty Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'FOC Controller/2', 'Id Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'FOC Controller/3', 'Iq Scope/1', 'autorouting', 'smart');

    case 'three_invertor'
        add_block('built-in/Subsystem', [model_name '/Three Invertor']);
        create_power_stage([model_name '/Three Invertor'], inv_params);
        set_param([model_name '/Three Invertor'], 'Position', [220 100 410 280]);

        add_constant_source(model_name, 'DutyABC', '[0.60; 0.40; 0.55]', [50 170 145 190]);
        add_scope_block([model_name '/Va Scope'], [500 85 570 125]);
        add_scope_block([model_name '/Vb Scope'], [500 155 570 195]);
        add_scope_block([model_name '/Vc Scope'], [500 225 570 265]);

        add_line(model_name, 'DutyABC/1', 'Three Invertor/1', 'autorouting', 'smart');
        add_line(model_name, 'Three Invertor/1', 'Va Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Three Invertor/2', 'Vb Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Three Invertor/3', 'Vc Scope/1', 'autorouting', 'smart');

    case 'motor'
        add_block('built-in/Subsystem', [model_name '/Motor']);
        create_pmsm_subsystem([model_name '/Motor'], motor_params);
        set_param([model_name '/Motor'], 'Position', [270 80 500 360]);

        add_sine_source(model_name, 'VaIn', num2str(inv_params.Vdc / 4), '2*pi*50', '0', [35 60 115 90]);
        add_sine_source(model_name, 'VbIn', num2str(inv_params.Vdc / 4), '2*pi*50', '-2*pi/3', [35 120 115 150]);
        add_sine_source(model_name, 'VcIn', num2str(inv_params.Vdc / 4), '2*pi*50', '2*pi/3', [35 180 115 210]);
        add_step_source(model_name, 'LoadIn', num2str(ref_params.load_step_time), '0', num2str(ref_params.load_torque), [30 250 115 280]);

        add_scope_block([model_name '/Current Scope'], [600 70 670 110]);
        add_scope_block([model_name '/Speed Scope'], [600 150 670 190]);
        add_scope_block([model_name '/Torque Scope'], [600 230 670 270]);
        add_scope_block([model_name '/Theta Scope'], [600 310 670 350]);

        add_line(model_name, 'VaIn/1', 'Motor/1', 'autorouting', 'smart');
        add_line(model_name, 'VbIn/1', 'Motor/2', 'autorouting', 'smart');
        add_line(model_name, 'VcIn/1', 'Motor/3', 'autorouting', 'smart');
        add_line(model_name, 'LoadIn/1', 'Motor/4', 'autorouting', 'smart');
        add_line(model_name, 'Motor/1', 'Current Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Motor/4', 'Speed Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Motor/5', 'Torque Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Motor/6', 'Theta Scope/1', 'autorouting', 'smart');

    case 'measure'
        add_block('built-in/Subsystem', [model_name '/Measure']);
        create_measurement_subsystem([model_name '/Measure'], motor_params);
        set_param([model_name '/Measure'], 'Position', [220 90 430 330]);

        add_constant_source(model_name, 'IaIn', '0.3', [45 70 95 90]);
        add_constant_source(model_name, 'IbIn', '-0.1', [45 115 95 135]);
        add_constant_source(model_name, 'IcIn', '-0.2', [45 160 95 180]);
        add_ramp_source(model_name, 'ThetaMIn', '2*pi*5', [30 205 100 235]);
        add_constant_source(model_name, 'OmegaMIn', '25.0', [45 255 95 275]);

        add_scope_block([model_name '/Current Scope'], [515 70 585 110]);
        add_scope_block([model_name '/Theta Scope'], [515 160 585 200]);
        add_scope_block([model_name '/Omega Scope'], [515 250 585 290]);

        add_line(model_name, 'IaIn/1', 'Measure/1', 'autorouting', 'smart');
        add_line(model_name, 'IbIn/1', 'Measure/2', 'autorouting', 'smart');
        add_line(model_name, 'IcIn/1', 'Measure/3', 'autorouting', 'smart');
        add_line(model_name, 'ThetaMIn/1', 'Measure/4', 'autorouting', 'smart');
        add_line(model_name, 'OmegaMIn/1', 'Measure/5', 'autorouting', 'smart');
        add_line(model_name, 'Measure/1', 'Current Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Measure/4', 'Theta Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Measure/5', 'Omega Scope/1', 'autorouting', 'smart');

    case 'scope'
        add_block('built-in/Subsystem', [model_name '/Scope']);
        create_scope_subsystem([model_name '/Scope']);
        set_param([model_name '/Scope'], 'Position', [230 100 460 320]);

        add_ramp_source(model_name, 'OmegaM', '100.0', [45 80 115 110]);
        add_sine_source(model_name, 'IdMeas', '1.0', '2*pi*25', '0', [40 145 115 175]);
        add_sine_source(model_name, 'IqMeas', '2.0', '2*pi*25', 'pi/2', [40 205 115 235]);
        add_step_source(model_name, 'TorqueE', '0.004', '0', '0.12', [35 270 115 300]);

        add_line(model_name, 'OmegaM/1', 'Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'IdMeas/1', 'Scope/2', 'autorouting', 'smart');
        add_line(model_name, 'IqMeas/1', 'Scope/3', 'autorouting', 'smart');
        add_line(model_name, 'TorqueE/1', 'Scope/4', 'autorouting', 'smart');

    otherwise
        error('Unsupported module name: %s', module_name);
end

save_generated_model(model_name, model_path);
end

function model_path = create_all_in_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params, inv_params);
model_path = initialize_model(model_name, sim_params, sim_params.t_end);

add_block('built-in/Subsystem', [model_name '/Signal In']);
create_signal_in_subsystem([model_name '/Signal In'], ref_params);
set_param([model_name '/Signal In'], 'Position', [60 90 240 250]);

add_ramp_source(model_name, 'CmdSpeedRef', num2str(ref_params.speed_ref / max(ref_params.speed_ramp_time, 1e-6), '%.16g'), [20 20 95 50]);
add_constant_source(model_name, 'CmdIdRef', num2str(ref_params.id_ref, '%.16g'), [20 60 95 80]);
add_step_source(model_name, 'CmdLoad', num2str(ref_params.load_step_time, '%.16g'), '0', num2str(ref_params.load_torque, '%.16g'), [10 95 95 125]);
add_constant_source(model_name, 'CmdThrottle', num2str(ref_params.throttle, '%.16g'), [20 135 95 155]);

add_block('built-in/Subsystem', [model_name '/Thro']);
create_thro_subsystem([model_name '/Thro'], ref_params, ctrl_params);
set_param([model_name '/Thro'], 'Position', [300 430 560 620]);

add_block('built-in/Subsystem', [model_name '/FOC Controller']);
create_foc_controller_subsystem([model_name '/FOC Controller'], ctrl_params, inv_params, motor_params, sim_params);
set_param([model_name '/FOC Controller'], 'Position', [300 80 560 390]);

add_constant_source(model_name, 'KpSpeedRt', num2str(ctrl_params.Kp_speed, '%.16g'), [210 20 290 40]);
add_constant_source(model_name, 'KiSpeedRt', num2str(ctrl_params.Ki_speed, '%.16g'), [210 50 290 70]);
add_constant_source(model_name, 'IqMaxSpeedRt', num2str(ctrl_params.iq_max, '%.16g'), [200 80 300 100]);
add_constant_source(model_name, 'KpIdRt', num2str(ctrl_params.Kp_id, '%.16g'), [210 110 290 130]);
add_constant_source(model_name, 'KiIdRt', num2str(ctrl_params.Ki_id, '%.16g'), [210 140 290 160]);
add_constant_source(model_name, 'KpIqRt', num2str(ctrl_params.Kp_iq, '%.16g'), [210 170 290 190]);
add_constant_source(model_name, 'KiIqRt', num2str(ctrl_params.Ki_iq, '%.16g'), [210 200 290 220]);
add_constant_source(model_name, 'IdMaxRt', num2str(ctrl_params.id_max, '%.16g'), [210 230 290 250]);
add_constant_source(model_name, 'IqMaxRt', num2str(ctrl_params.iq_max, '%.16g'), [210 260 290 280]);

add_block('built-in/Subsystem', [model_name '/Three Invertor']);
create_power_stage([model_name '/Three Invertor'], inv_params);
set_param([model_name '/Three Invertor'], 'Position', [640 145 820 315]);

add_block('built-in/Subsystem', [model_name '/Motor']);
create_pmsm_subsystem([model_name '/Motor'], motor_params);
set_param([model_name '/Motor'], 'Position', [920 100 1160 380]);

add_block('built-in/Subsystem', [model_name '/Measure']);
create_measurement_subsystem([model_name '/Measure'], motor_params);
set_param([model_name '/Measure'], 'Position', [930 440 1160 650]);

zoh_ts = num2str(sim_params.Ts_control, '%.16g');
add_block('simulink/Discrete/Zero-Order Hold', [model_name '/ZOH_ia'], 'SampleTime', zoh_ts, 'Position', [1185 470 1245 490]);
add_block('simulink/Discrete/Zero-Order Hold', [model_name '/ZOH_ib'], 'SampleTime', zoh_ts, 'Position', [1185 505 1245 525]);
add_block('simulink/Discrete/Zero-Order Hold', [model_name '/ZOH_ic'], 'SampleTime', zoh_ts, 'Position', [1185 540 1245 560]);
add_block('simulink/Discrete/Zero-Order Hold', [model_name '/ZOH_theta_e'], 'SampleTime', zoh_ts, 'Position', [1185 575 1245 595]);
add_block('simulink/Discrete/Zero-Order Hold', [model_name '/ZOH_omega_m'], 'SampleTime', zoh_ts, 'Position', [1185 610 1245 630]);

add_block('built-in/Subsystem', [model_name '/Scope']);
create_scope_subsystem([model_name '/Scope']);
set_param([model_name '/Scope'], 'Position', [1270 150 1470 360]);

add_line(model_name, 'CmdSpeedRef/1', 'Signal In/1', 'autorouting', 'smart');
add_line(model_name, 'CmdIdRef/1', 'Signal In/2', 'autorouting', 'smart');
add_line(model_name, 'CmdLoad/1', 'Signal In/3', 'autorouting', 'smart');
add_line(model_name, 'CmdThrottle/1', 'Signal In/4', 'autorouting', 'smart');

add_line(model_name, 'Signal In/1', 'FOC Controller/6', 'autorouting', 'smart');
add_line(model_name, 'Signal In/2', 'FOC Controller/7', 'autorouting', 'smart');
add_line(model_name, 'Signal In/3', 'Motor/4', 'autorouting', 'smart');
add_line(model_name, 'Signal In/1', 'Thro/2', 'autorouting', 'smart');
add_line(model_name, 'Signal In/4', 'Thro/3', 'autorouting', 'smart');
add_line(model_name, 'Thro/1', 'FOC Controller/8', 'autorouting', 'smart');
add_line(model_name, 'KpSpeedRt/1', 'FOC Controller/9', 'autorouting', 'smart');
add_line(model_name, 'KiSpeedRt/1', 'FOC Controller/10', 'autorouting', 'smart');
add_line(model_name, 'IqMaxSpeedRt/1', 'FOC Controller/11', 'autorouting', 'smart');
add_line(model_name, 'KpIdRt/1', 'FOC Controller/12', 'autorouting', 'smart');
add_line(model_name, 'KiIdRt/1', 'FOC Controller/13', 'autorouting', 'smart');
add_line(model_name, 'KpIqRt/1', 'FOC Controller/14', 'autorouting', 'smart');
add_line(model_name, 'KiIqRt/1', 'FOC Controller/15', 'autorouting', 'smart');
add_line(model_name, 'IdMaxRt/1', 'FOC Controller/16', 'autorouting', 'smart');
add_line(model_name, 'IqMaxRt/1', 'FOC Controller/17', 'autorouting', 'smart');

add_line(model_name, 'FOC Controller/1', 'Three Invertor/1', 'autorouting', 'smart');

add_line(model_name, 'Three Invertor/1', 'Motor/1', 'autorouting', 'smart');
add_line(model_name, 'Three Invertor/2', 'Motor/2', 'autorouting', 'smart');
add_line(model_name, 'Three Invertor/3', 'Motor/3', 'autorouting', 'smart');

add_line(model_name, 'Motor/1', 'Measure/1', 'autorouting', 'smart');
add_line(model_name, 'Motor/2', 'Measure/2', 'autorouting', 'smart');
add_line(model_name, 'Motor/3', 'Measure/3', 'autorouting', 'smart');
add_line(model_name, 'Motor/6', 'Measure/4', 'autorouting', 'smart');
add_line(model_name, 'Motor/4', 'Measure/5', 'autorouting', 'smart');

add_line(model_name, 'Measure/1', 'ZOH_ia/1', 'autorouting', 'smart');
add_line(model_name, 'Measure/2', 'ZOH_ib/1', 'autorouting', 'smart');
add_line(model_name, 'Measure/3', 'ZOH_ic/1', 'autorouting', 'smart');
add_line(model_name, 'Measure/4', 'ZOH_theta_e/1', 'autorouting', 'smart');
add_line(model_name, 'Measure/5', 'ZOH_omega_m/1', 'autorouting', 'smart');

add_line(model_name, 'ZOH_ia/1', 'FOC Controller/1', 'autorouting', 'smart');
add_line(model_name, 'ZOH_ib/1', 'FOC Controller/2', 'autorouting', 'smart');
add_line(model_name, 'ZOH_ic/1', 'FOC Controller/3', 'autorouting', 'smart');
add_line(model_name, 'ZOH_theta_e/1', 'FOC Controller/4', 'autorouting', 'smart');
add_line(model_name, 'ZOH_omega_m/1', 'FOC Controller/5', 'autorouting', 'smart');
add_line(model_name, 'ZOH_omega_m/1', 'Thro/1', 'autorouting', 'smart');

add_line(model_name, 'Motor/4', 'Scope/1', 'autorouting', 'smart');
add_line(model_name, 'FOC Controller/2', 'Scope/2', 'autorouting', 'smart');
add_line(model_name, 'FOC Controller/3', 'Scope/3', 'autorouting', 'smart');
add_line(model_name, 'Motor/5', 'Scope/4', 'autorouting', 'smart');

save_generated_model(model_name, model_path);
end

function ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params, inv_params)
if ~isfield(ctrl_params, 'auto_tune_current') || isempty(ctrl_params.auto_tune_current)
    ctrl_params.auto_tune_current = true;
end

if ~isfield(ctrl_params, 'auto_tune_speed') || isempty(ctrl_params.auto_tune_speed)
    ctrl_params.auto_tune_speed = true;
end

if ~isfield(ctrl_params, 'enable_internal_speed_loop') || isempty(ctrl_params.enable_internal_speed_loop)
    ctrl_params.enable_internal_speed_loop = true;
end

if ~isfield(ctrl_params, 'omega_ci') || isempty(ctrl_params.omega_ci) || ctrl_params.omega_ci <= 0
    omega_ci = 2*pi*(inv_params.fsw / 10.0);
else
    omega_ci = ctrl_params.omega_ci;
end

if ~isfield(ctrl_params, 'omega_cs') || isempty(ctrl_params.omega_cs) || ctrl_params.omega_cs <= 0
    omega_cs = omega_ci / 10.0;
else
    omega_cs = ctrl_params.omega_cs;
end

ctrl_params.omega_ci = omega_ci;
ctrl_params.omega_cs = omega_cs;

Rs = max(motor_params.Rs, 1e-12);
Ld = max(motor_params.Ld, 1e-12);
Lq = max(motor_params.Lq, 1e-12);

if ctrl_params.auto_tune_current
    % Current-loop PI: zero cancels electrical pole at Rs/L.
    ctrl_params.Kp_id = Ld * omega_ci;
    ctrl_params.Ki_id = Rs * omega_ci;
    ctrl_params.Kp_iq = Lq * omega_ci;
    ctrl_params.Ki_iq = Rs * omega_ci;
end

J = max(motor_params.J, 1e-12);
B = max(motor_params.B, 1e-12);
Kt = max(1.5 * motor_params.p * motor_params.flux_pm, 1e-12);

% Speed-loop PI: zero cancels mechanical pole at B/J.
if ctrl_params.auto_tune_speed
    ctrl_params.Kp_speed = (J * omega_cs) / Kt;
    ctrl_params.Ki_speed = (B * omega_cs) / Kt;
end
end

function result = validate_model(model_name, stop_time)
result = struct('model_name', model_name, 'saved', false, 'compiled', false, ...
    'simulated', false, 'message', '');

ensure_generated_models_path();
model_path = get_model_path(model_name);
result.saved = isfile(model_path);
if ~result.saved
    result.message = sprintf('Model file does not exist: %s', model_path);
    return;
end

try
    load_system(model_name);
    set_param(model_name, 'SimulationCommand', 'update');
    result.compiled = true;

    sim(model_name, 'StopTime', num2str(stop_time));
    result.simulated = true;
    result.message = 'ok';
catch model_error
    result.message = model_error.message;
end

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end

function model_path = initialize_model(model_name, sim_params, stop_time)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

ensure_generated_models_path();
model_path = get_model_path(model_name);
if isfile(model_path)
    delete(model_path);
end

new_system(model_name);
set_param(model_name, ...
    'Solver', sim_params.solver, ...
    'StopTime', num2str(stop_time), ...
    'MaxStep', num2str(sim_params.max_step), ...
    'RelTol', '1e-4', ...
    'AbsTol', '1e-6');
end

function save_generated_model(model_name, model_path)
model_dir = fileparts(model_path);
if exist(model_dir, 'dir') ~= 7
    mkdir(model_dir);
end
try
    save_system(model_name, model_path);
catch
    % MATLAB MCP can fail while generating SLX thumbnails for Simscape-heavy models.
    % Retrying with ExportToXML avoids the thumbnail callback path while still producing an SLX file.
    save_system(model_name, model_path, 'ExportToXML', true);
end
fprintf('Model saved to: %s\n', model_path);
end

function ensure_generated_models_path()
script_dir = fileparts(mfilename('fullpath'));
model_dir = fullfile(script_dir, '..', 'models');
addpath(model_dir);
rehash path;
end

function model_path = get_model_path(model_name)
script_dir = fileparts(mfilename('fullpath'));
model_path = fullfile(script_dir, '..', 'models', [model_name '.slx']);
end

function add_constant_source(model_name, block_name, value, position)
add_block('built-in/Constant', [model_name '/' block_name], ...
    'Value', value, ...
    'Position', position);
end

function add_step_source(model_name, block_name, step_time, before_value, after_value, position)
add_block('simulink/Sources/Step', [model_name '/' block_name], ...
    'Time', step_time, ...
    'Before', before_value, ...
    'After', after_value, ...
    'Position', position);
end

function add_ramp_source(model_name, block_name, slope, position)
add_block('simulink/Sources/Ramp', [model_name '/' block_name], ...
    'Slope', slope, ...
    'Position', position);
end

function add_sine_source(model_name, block_name, amplitude, frequency, phase, position)
add_block('simulink/Sources/Sine Wave', [model_name '/' block_name], ...
    'Amplitude', amplitude, ...
    'Frequency', frequency, ...
    'Phase', phase, ...
    'Position', position);
end

function add_scope_block(block_path, position)
add_block('simulink/Sinks/Scope', block_path, 'Position', position);
end

function create_power_stage(subsys, inv_params)
add_block('built-in/Inport',  [subsys '/duty_abc'], 'Port', '1');
add_block('built-in/Outport', [subsys '/Va'], 'Port', '1');
add_block('built-in/Outport', [subsys '/Vb'], 'Port', '2');
add_block('built-in/Outport', [subsys '/Vc'], 'Port', '3');

add_block('built-in/Constant', [subsys '/Vdc'], ...
    'Value', num2str(inv_params.Vdc), ...
    'Position', [80 150 130 170]);

if isfield(inv_params, 'Tsw') && ~isempty(inv_params.Tsw)
    Ts_power = inv_params.Tsw;
else
    Ts_power = 1 / inv_params.fsw;
end

add_block('simulink/Discrete/Unit Delay', [subsys '/Duty Delay'], ...
    'SampleTime', num2str(Ts_power, '%.16g'), ...
    'InitialCondition', '0.5', ...
    'Position', [130 80 165 110]);

add_block('simulink/Math Operations/Product', [subsys '/Modulator'], ...
    'Inputs', '2', ...
    'Multiplication', 'Element-wise(.*)', ...
    'Position', [220 80 270 130]);

add_block('simulink/Signal Routing/Demux', [subsys '/Demux'], ...
    'Outputs', '3', ...
    'Position', [320 70 330 150]);

add_line(subsys, 'duty_abc/1', 'Duty Delay/1');
add_line(subsys, 'Duty Delay/1', 'Modulator/1');
add_line(subsys, 'Vdc/1', 'Modulator/2');
add_line(subsys, 'Modulator/1', 'Demux/1');
add_line(subsys, 'Demux/1', 'Va/1');
add_line(subsys, 'Demux/2', 'Vb/1');
add_line(subsys, 'Demux/3', 'Vc/1');
end

function create_foc_controller_subsystem(subsys, ctrl_params, inv_params, motor_params, sim_params)
add_block('built-in/Inport', [subsys '/ia'], 'Port', '1');
add_block('built-in/Inport', [subsys '/ib'], 'Port', '2');
add_block('built-in/Inport', [subsys '/ic'], 'Port', '3');
add_block('built-in/Inport', [subsys '/theta_e'], 'Port', '4');
add_block('built-in/Inport', [subsys '/omega_m'], 'Port', '5');
add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
add_block('built-in/Inport', [subsys '/id_ref'], 'Port', '7');
add_block('built-in/Inport', [subsys '/iq_ref_cmd'], 'Port', '8');
add_block('built-in/Inport', [subsys '/kp_speed_rt'], 'Port', '9');
add_block('built-in/Inport', [subsys '/ki_speed_rt'], 'Port', '10');
add_block('built-in/Inport', [subsys '/iq_max_speed_rt'], 'Port', '11');
add_block('built-in/Inport', [subsys '/kp_id_rt'], 'Port', '12');
add_block('built-in/Inport', [subsys '/ki_id_rt'], 'Port', '13');
add_block('built-in/Inport', [subsys '/kp_iq_rt'], 'Port', '14');
add_block('built-in/Inport', [subsys '/ki_iq_rt'], 'Port', '15');
add_block('built-in/Inport', [subsys '/id_max_rt'], 'Port', '16');
add_block('built-in/Inport', [subsys '/iq_max_rt'], 'Port', '17');

add_block('built-in/Outport', [subsys '/duty_abc'], 'Port', '1');
add_block('built-in/Outport', [subsys '/id_meas'], 'Port', '2');
add_block('built-in/Outport', [subsys '/iq_meas'], 'Port', '3');

if isfield(sim_params, 'Ts_control') && ~isempty(sim_params.Ts_control)
    Ts_control = sim_params.Ts_control;
elseif isfield(inv_params, 'Tsw') && ~isempty(inv_params.Tsw)
    Ts_control = inv_params.Tsw;
else
    Ts_control = 1 / inv_params.fsw;
end

if isfield(sim_params, 'Ts_speed') && ~isempty(sim_params.Ts_speed)
    Ts_speed = sim_params.Ts_speed;
else
    Ts_speed = 10 * Ts_control;
end

speed_sfun_params = strjoin({ ...
    num2str(Ts_speed, '%.16g'), ...
    num2str(ctrl_params.Kp_speed, '%.16g'), ...
    num2str(ctrl_params.Ki_speed, '%.16g'), ...
    num2str(ctrl_params.iq_max, '%.16g') ...
}, ', ');

sfun_params = strjoin({ ...
    num2str(Ts_control, '%.16g'), ...
    num2str(inv_params.Vdc, '%.16g'), ...
    num2str(ctrl_params.omega_ci, '%.16g'), ...
    num2str(motor_params.Rs, '%.16g'), ...
    num2str(motor_params.Ld, '%.16g'), ...
    num2str(motor_params.Lq, '%.16g'), ...
    num2str(motor_params.flux_pm, '%.16g'), ...
    num2str(motor_params.p, '%.16g'), ...
    num2str(ctrl_params.iq_max, '%.16g'), ...
    num2str(ctrl_params.id_max, '%.16g'), ...
    num2str(ctrl_params.Kp_id, '%.16g'), ...
    num2str(ctrl_params.Ki_id, '%.16g'), ...
    num2str(ctrl_params.Kp_iq, '%.16g'), ...
    num2str(ctrl_params.Ki_iq, '%.16g'), ...
    num2str(ctrl_params.auto_tune_current, '%.16g') ...
}, ', ');

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_speed_inputs'], ...
    'Inputs', '5', 'Position', [128 205 133 320]);
add_block('simulink/User-Defined Functions/S-Function', [subsys '/Speed Core'], ...
    'FunctionName', 'sfun_speed_controller', ...
    'Parameters', speed_sfun_params, ...
    'Position', [185 230 320 280]);
add_block('simulink/Math Operations/Sum', [subsys '/Iq Sum'], ...
    'Inputs', '++', ...
    'Position', [355 242 380 268]);
add_block('simulink/Discontinuities/Saturation', [subsys '/Iq Ref Sat'], ...
    'UpperLimit', num2str(ctrl_params.iq_max, '%.16g'), ...
    'LowerLimit', num2str(-ctrl_params.iq_max, '%.16g'), ...
    'Position', [410 235 475 275]);

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_inputs'], ...
    'Inputs', '13', 'Position', [135 40 140 340]);
add_block('simulink/User-Defined Functions/S-Function', [subsys '/FOC Core'], ...
    'FunctionName', 'sfun_foc_controller', ...
    'Parameters', sfun_params, ...
    'Position', [205 120 360 200]);
add_block('simulink/Signal Routing/Demux', [subsys '/Demux_outputs'], ...
    'Outputs', '5', 'Position', [410 118 415 202]);
add_block('simulink/Signal Routing/Mux', [subsys '/Mux_duty'], ...
    'Inputs', '3', 'Position', [455 70 460 150]);

add_line(subsys, 'ia/1', 'Mux_inputs/1');
add_line(subsys, 'ib/1', 'Mux_inputs/2');
add_line(subsys, 'ic/1', 'Mux_inputs/3');
add_line(subsys, 'theta_e/1', 'Mux_inputs/4');
add_line(subsys, 'omega_m/1', 'Mux_inputs/5');
add_line(subsys, 'id_ref/1', 'Mux_inputs/6');
add_line(subsys, 'speed_ref/1', 'Mux_speed_inputs/1');
add_line(subsys, 'omega_m/1', 'Mux_speed_inputs/2');
add_line(subsys, 'kp_speed_rt/1', 'Mux_speed_inputs/3');
add_line(subsys, 'ki_speed_rt/1', 'Mux_speed_inputs/4');
add_line(subsys, 'iq_max_speed_rt/1', 'Mux_speed_inputs/5');
add_line(subsys, 'Mux_speed_inputs/1', 'Speed Core/1');
add_line(subsys, 'Speed Core/1', 'Iq Sum/1');
add_line(subsys, 'iq_ref_cmd/1', 'Iq Sum/2');
add_line(subsys, 'Iq Sum/1', 'Iq Ref Sat/1');
add_line(subsys, 'Iq Ref Sat/1', 'Mux_inputs/7');
add_line(subsys, 'kp_id_rt/1', 'Mux_inputs/8');
add_line(subsys, 'ki_id_rt/1', 'Mux_inputs/9');
add_line(subsys, 'kp_iq_rt/1', 'Mux_inputs/10');
add_line(subsys, 'ki_iq_rt/1', 'Mux_inputs/11');
add_line(subsys, 'id_max_rt/1', 'Mux_inputs/12');
add_line(subsys, 'iq_max_rt/1', 'Mux_inputs/13');

add_line(subsys, 'Mux_inputs/1', 'FOC Core/1');
add_line(subsys, 'FOC Core/1', 'Demux_outputs/1');
add_line(subsys, 'Demux_outputs/1', 'Mux_duty/1');
add_line(subsys, 'Demux_outputs/2', 'Mux_duty/2');
add_line(subsys, 'Demux_outputs/3', 'Mux_duty/3');
add_line(subsys, 'Mux_duty/1', 'duty_abc/1');
add_line(subsys, 'Demux_outputs/4', 'id_meas/1');
add_line(subsys, 'Demux_outputs/5', 'iq_meas/1');
end

function create_pmsm_subsystem(subsys, motor_params)
add_block('built-in/Inport', [subsys '/Va'], 'Port', '1');
add_block('built-in/Inport', [subsys '/Vb'], 'Port', '2');
add_block('built-in/Inport', [subsys '/Vc'], 'Port', '3');
add_block('built-in/Inport', [subsys '/Te_load'], 'Port', '4');

add_block('built-in/Outport', [subsys '/ia'], 'Port', '1');
add_block('built-in/Outport', [subsys '/ib'], 'Port', '2');
add_block('built-in/Outport', [subsys '/ic'], 'Port', '3');
add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '4');
add_block('built-in/Outport', [subsys '/Te'], 'Port', '5');
add_block('built-in/Outport', [subsys '/theta_m'], 'Port', '6');

% Simulink to physical signal adapters for three-phase voltage and load torque.
add_block('simulink/Signal Routing/Mux', [subsys '/MuxVabc'], ...
    'Inputs', '3', 'Position', [90 65 95 155]);
add_block('simulink/Sinks/Terminator', [subsys '/LoadUnused'], ...
    'Position', [115 210 135 230]);

add_reference_block_with_fallback([subsys '/Vabc2PS'], {
    'nesl_utility/Simulink-PS Converter'
}, [170 90 220 130]);

set_block_params_safe([subsys '/Vabc2PS'], struct('Unit', 'V'));

% Core Simscape machine and converters modeled after local PMSM example.
add_reference_block_with_fallback([subsys '/Solver Configuration'], {
    'nesl_utility/Solver Configuration'
}, [40 300 95 335]);
add_reference_block_with_fallback([subsys '/Electrical Reference'], {
    'fl_lib/Electrical/Electrical Elements/Electrical Reference'
}, [155 310 205 340]);
add_reference_block_with_fallback([subsys '/Mechanical Reference'], {
    'fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference'
}, [530 310 590 340]);
add_reference_block_with_fallback([subsys '/Rotor Inertia'], {
    'fl_lib/Mechanical/Rotational Elements/Inertia'
}, [640 285 700 335]);
set_block_params_safe([subsys '/Rotor Inertia'], struct('inertia', '1e-9'));
add_reference_block_with_fallback([subsys '/Rotational Friction'], {
    'fl_lib/Mechanical/Rotational Elements/Rotational Friction'
}, [640 185 715 245]);
set_block_params_safe([subsys '/Rotational Friction'], struct( ...
    'visc_coef', num2str(motor_params.B, '%.6g'), ...
    'visc_coef_unit', 'N*m/(rad/s)', ...
    'brkwy_trq', '1e-9', ...
    'Col_trq', '1e-9'));

add_reference_block_with_fallback([subsys '/ThreePhase Controlled Voltage'], {
    'ee_lib/Sources/Controlled Voltage Source (Three-Phase)', ...
    'ee_lib/Sources & Subsystems/Controlled Voltage Source (Three-Phase)', ...
    'ee_lib/Sources/Three-Phase Sources/Controlled Voltage Source (Three-Phase)'
}, [250 90 340 180]);

add_reference_block_with_fallback([subsys '/Current Sensor'], {
    'ee_lib/Sensors & Transducers/Current Sensor (Three-Phase)'
}, [370 95 455 175]);

add_reference_block_with_fallback([subsys '/PMSM'], {
    'ee_lib/Electromechanical/Permanent Magnet/PMSM'
}, [490 85 595 185]);

add_reference_block_with_fallback([subsys '/Ideal Torque Sensor'], {
    'fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor'
}, [640 85 710 150]);
add_reference_block_with_fallback([subsys '/Motion Sensor'], {
    'fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor'
}, [760 85 825 155]);

add_reference_block_with_fallback([subsys '/CurrentPS2SL'], {
    'nesl_utility/PS-Simulink Converter'
}, [500 220 560 255]);
add_reference_block_with_fallback([subsys '/TorquePS2SL'], {
    'nesl_utility/PS-Simulink Converter'
}, [700 220 760 255]);
add_reference_block_with_fallback([subsys '/OmegaPS2SL'], {
    'nesl_utility/PS-Simulink Converter'
}, [790 220 850 255]);
add_reference_block_with_fallback([subsys '/ThetaPS2SL'], {
    'nesl_utility/PS-Simulink Converter'
}, [860 220 920 255]);

set_block_params_safe([subsys '/CurrentPS2SL'], struct('Unit', 'A'));
set_block_params_safe([subsys '/TorquePS2SL'], struct('Unit', 'N*m'));
set_block_params_safe([subsys '/OmegaPS2SL'], struct('Unit', 'rad/s'));
set_block_params_safe([subsys '/ThetaPS2SL'], struct('Unit', 'rad'));

add_block('simulink/Signal Routing/Demux', [subsys '/DemuxIabc'], ...
    'Outputs', '3', 'Position', [600 210 605 280]);

set_block_params_safe([subsys '/PMSM'], struct( ...
    'nPolePairs', num2str(motor_params.p, '%.6g'), ...
    'pm_flux_linkage', num2str(motor_params.flux_pm, '%.6g'), ...
    'Ld', num2str(motor_params.Ld, '%.6g'), ...
    'Lq', num2str(motor_params.Lq, '%.6g'), ...
    'Rs', num2str(motor_params.Rs, '%.6g'), ...
    'J', num2str(motor_params.J, '%.6g'), ...
    'lam', num2str(motor_params.B, '%.6g'), ...
    'angular_velocity_specify', 'on', ...
    'angular_velocity', '0', ...
    'angular_velocity_unit', 'rpm', ...
    'angular_position_specify', 'on', ...
    'angular_position', '0', ...
    'angular_position_unit', 'rad'));

add_line(subsys, 'Va/1', 'MuxVabc/1');
add_line(subsys, 'Vb/1', 'MuxVabc/2');
add_line(subsys, 'Vc/1', 'MuxVabc/3');
add_line(subsys, 'MuxVabc/1', 'Vabc2PS/1');
add_line(subsys, 'Te_load/1', 'LoadUnused/1');

add_line(subsys, 'CurrentPS2SL/1', 'DemuxIabc/1');
add_line(subsys, 'DemuxIabc/1', 'ia/1');
add_line(subsys, 'DemuxIabc/2', 'ib/1');
add_line(subsys, 'DemuxIabc/3', 'ic/1');
add_line(subsys, 'OmegaPS2SL/1', 'omega_m/1');
add_line(subsys, 'TorquePS2SL/1', 'Te/1');
add_line(subsys, 'ThetaPS2SL/1', 'theta_m/1');

ph_vabc2ps = get_param([subsys '/Vabc2PS'], 'PortHandles');
ph_solver = get_param([subsys '/Solver Configuration'], 'PortHandles');
ph_eref = get_param([subsys '/Electrical Reference'], 'PortHandles');
ph_vsrc = get_param([subsys '/ThreePhase Controlled Voltage'], 'PortHandles');
ph_isens = get_param([subsys '/Current Sensor'], 'PortHandles');
ph_pmsm = get_param([subsys '/PMSM'], 'PortHandles');
ph_tsens = get_param([subsys '/Ideal Torque Sensor'], 'PortHandles');
ph_msens = get_param([subsys '/Motion Sensor'], 'PortHandles');
ph_mref = get_param([subsys '/Mechanical Reference'], 'PortHandles');
ph_fric = get_param([subsys '/Rotational Friction'], 'PortHandles');
ph_i2sl = get_param([subsys '/CurrentPS2SL'], 'PortHandles');
ph_t2sl = get_param([subsys '/TorquePS2SL'], 'PortHandles');
ph_w2sl = get_param([subsys '/OmegaPS2SL'], 'PortHandles');
ph_th2sl = get_param([subsys '/ThetaPS2SL'], 'PortHandles');

% Electrical network: control signal -> 3-phase source -> current sensor -> PMSM.
add_line(subsys, ph_vabc2ps.RConn(1), ph_vsrc.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_eref.LConn(1), ph_vsrc.LConn(2), 'autorouting', 'smart');
add_line(subsys, ph_solver.RConn(1), ph_vsrc.LConn(2), 'autorouting', 'smart');
add_line(subsys, ph_vsrc.RConn(1), ph_isens.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_isens.RConn(2), ph_pmsm.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_isens.RConn(1), ph_i2sl.LConn(1), 'autorouting', 'smart');

% Mechanical network: measure shaft motion and torque against reference.
add_line(subsys, ph_pmsm.RConn(1), ph_tsens.LConn(1), 'autorouting', 'smart');
ph_inertia = get_param([subsys '/Rotor Inertia'], 'PortHandles');
add_line(subsys, ph_tsens.RConn(1), ph_inertia.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_tsens.RConn(1), ph_msens.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_tsens.RConn(1), ph_fric.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_msens.RConn(1), ph_mref.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_fric.RConn(1), ph_mref.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_pmsm.RConn(2), ph_mref.LConn(1), 'autorouting', 'smart');

add_line(subsys, ph_tsens.RConn(2), ph_t2sl.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_msens.RConn(2), ph_w2sl.LConn(1), 'autorouting', 'smart');
add_line(subsys, ph_msens.RConn(3), ph_th2sl.LConn(1), 'autorouting', 'smart');
end

function create_measurement_subsystem(subsys, motor_params)
add_block('built-in/Inport', [subsys '/ia'], 'Port', '1');
add_block('built-in/Inport', [subsys '/ib'], 'Port', '2');
add_block('built-in/Inport', [subsys '/ic'], 'Port', '3');
add_block('built-in/Inport', [subsys '/theta_m'], 'Port', '4');
add_block('built-in/Inport', [subsys '/omega_m_in'], 'Port', '5');

add_block('built-in/Outport', [subsys '/ia_meas'], 'Port', '1');
add_block('built-in/Outport', [subsys '/ib_meas'], 'Port', '2');
add_block('built-in/Outport', [subsys '/ic_meas'], 'Port', '3');
add_block('built-in/Outport', [subsys '/theta_e'], 'Port', '4');
add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '5');

add_line(subsys, 'ia/1', 'ia_meas/1');
add_line(subsys, 'ib/1', 'ib_meas/1');
add_line(subsys, 'ic/1', 'ic_meas/1');

add_block('simulink/Math Operations/Gain', [subsys '/PoleP'], ...
    'Gain', num2str(motor_params.p), ...
    'Position', [200 180 250 210]);
add_line(subsys, 'theta_m/1', 'PoleP/1');
add_line(subsys, 'PoleP/1', 'theta_e/1');
add_line(subsys, 'omega_m_in/1', 'omega_m/1');
end

function add_reference_block_with_fallback(block_path, source_candidates, position)
last_error = '';
for idx = 1:numel(source_candidates)
    src = source_candidates{idx};
    try
        add_block(src, block_path, 'Position', position);
        return;
    catch err
        last_error = err.message;
    end
end
error('Failed to add reference block %s. Last error: %s', block_path, last_error);
end

function set_block_params_safe(block_path, params)
fields = fieldnames(params);
for idx = 1:numel(fields)
    key = fields{idx};
    value = params.(key);
    try
        set_param(block_path, key, value);
    catch
        % Keep compatibility across MATLAB releases with parameter name drift.
    end
end
end

function create_signal_in_subsystem(subsys, ref_params)
if ~isfield(ref_params, 'use_external_inputs') || isempty(ref_params.use_external_inputs)
    use_external_inputs = false;
else
    use_external_inputs = logical(ref_params.use_external_inputs);
end

speed_ramp_time = max(ref_params.speed_ramp_time, 1e-6);

add_block('built-in/Inport', [subsys '/speed_ref_ext'], 'Port', '1');
add_block('built-in/Inport', [subsys '/id_ref_ext'], 'Port', '2');
add_block('built-in/Inport', [subsys '/Te_load_ext'], 'Port', '3');
add_block('built-in/Inport', [subsys '/throttle_ext'], 'Port', '4');

add_block('built-in/Outport', [subsys '/speed_ref'], 'Port', '1');
add_block('built-in/Outport', [subsys '/id_ref'], 'Port', '2');
add_block('built-in/Outport', [subsys '/Te_load'], 'Port', '3');
add_block('built-in/Outport', [subsys '/throttle'], 'Port', '4');

add_block('simulink/Sources/Ramp', [subsys '/Speed Ramp'], ...
    'Slope', num2str(ref_params.speed_ref / speed_ramp_time, '%.16g'), ...
    'Position', [50 30 120 60]);
add_block('simulink/Discontinuities/Saturation', [subsys '/Speed Sat'], ...
    'UpperLimit', num2str(ref_params.speed_ref), ...
    'LowerLimit', '0', ...
    'Position', [160 30 220 60]);
add_block('built-in/Constant', [subsys '/Id Ref'], ...
    'Value', num2str(ref_params.id_ref), ...
    'Position', [100 100 150 120]);
add_block('simulink/Sources/Step', [subsys '/Load Step'], ...
    'Time', num2str(ref_params.load_step_time), ...
    'Before', '0', ...
    'After', num2str(ref_params.load_torque), ...
    'Position', [70 170 140 200]);
add_block('built-in/Constant', [subsys '/Throttle'], ...
    'Value', num2str(ref_params.throttle), ...
    'Position', [100 235 150 255]);

add_block('built-in/Constant', [subsys '/UseExternalInputs'], ...
    'Value', num2str(double(use_external_inputs), '%.16g'), ...
    'Position', [230 270 290 290]);

add_block('simulink/Signal Routing/Switch', [subsys '/Speed Select'], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5', ...
    'Position', [270 25 320 65]);
add_block('simulink/Signal Routing/Switch', [subsys '/Id Select'], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5', ...
    'Position', [270 95 320 135]);
add_block('simulink/Signal Routing/Switch', [subsys '/Load Select'], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5', ...
    'Position', [270 165 320 205]);
add_block('simulink/Signal Routing/Switch', [subsys '/Throttle Select'], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5', ...
    'Position', [270 230 320 270]);

add_line(subsys, 'Speed Ramp/1', 'Speed Sat/1');
add_line(subsys, 'speed_ref_ext/1', 'Speed Select/1');
add_line(subsys, 'UseExternalInputs/1', 'Speed Select/2');
add_line(subsys, 'Speed Sat/1', 'Speed Select/3');
add_line(subsys, 'Speed Select/1', 'speed_ref/1');

add_line(subsys, 'id_ref_ext/1', 'Id Select/1');
add_line(subsys, 'UseExternalInputs/1', 'Id Select/2');
add_line(subsys, 'Id Ref/1', 'Id Select/3');
add_line(subsys, 'Id Select/1', 'id_ref/1');

add_line(subsys, 'Te_load_ext/1', 'Load Select/1');
add_line(subsys, 'UseExternalInputs/1', 'Load Select/2');
add_line(subsys, 'Load Step/1', 'Load Select/3');
add_line(subsys, 'Load Select/1', 'Te_load/1');

add_line(subsys, 'throttle_ext/1', 'Throttle Select/1');
add_line(subsys, 'UseExternalInputs/1', 'Throttle Select/2');
add_line(subsys, 'Throttle/1', 'Throttle Select/3');
add_line(subsys, 'Throttle Select/1', 'throttle/1');
end

function create_thro_subsystem(subsys, ref_params, ctrl_params)
add_block('built-in/Inport', [subsys '/current_speed'], 'Port', '1');
add_block('built-in/Inport', [subsys '/target_speed'], 'Port', '2');
add_block('built-in/Inport', [subsys '/throttle'], 'Port', '3');

add_block('built-in/Outport', [subsys '/iq_ref_ff'], 'Port', '1');

if isfield(ref_params, 'throttle_iq_max') && ~isempty(ref_params.throttle_iq_max)
    iq_max = max(ref_params.throttle_iq_max, 0.0);
else
    iq_max = max(ctrl_params.iq_max, 0.0);
end
add_block('simulink/Discontinuities/Saturation', [subsys '/Throttle Sat'], ...
    'UpperLimit', '1', ...
    'LowerLimit', '0', ...
    'Position', [110 150 165 180]);
add_block('simulink/Math Operations/Gain', [subsys '/Throttle2Iq'], ...
    'Gain', num2str(iq_max, '%.16g'), ...
    'Position', [220 150 285 180]);
add_block('simulink/Discontinuities/Saturation', [subsys '/Iq Sat'], ...
    'UpperLimit', num2str(iq_max), ...
    'LowerLimit', num2str(-iq_max), ...
    'Position', [330 145 390 185]);
add_block('simulink/Sinks/Terminator', [subsys '/UnusedSpeed'], ...
    'Position', [120 30 140 50]);
add_block('simulink/Sinks/Terminator', [subsys '/UnusedTarget'], ...
    'Position', [120 80 140 100]);

add_line(subsys, 'current_speed/1', 'UnusedSpeed/1');
add_line(subsys, 'target_speed/1', 'UnusedTarget/1');
add_line(subsys, 'throttle/1', 'Throttle Sat/1');
add_line(subsys, 'Throttle Sat/1', 'Throttle2Iq/1');
add_line(subsys, 'Throttle2Iq/1', 'Iq Sat/1');
add_line(subsys, 'Iq Sat/1', 'iq_ref_ff/1');
end

function create_scope_subsystem(subsys)
add_block('built-in/Inport', [subsys '/omega_m'], 'Port', '1');
add_block('built-in/Inport', [subsys '/id_meas'], 'Port', '2');
add_block('built-in/Inport', [subsys '/iq_meas'], 'Port', '3');
add_block('built-in/Inport', [subsys '/torque_e'], 'Port', '4');

add_block('simulink/Signal Routing/Mux', [subsys '/CurrentMux'], ...
    'Inputs', '2', 'Position', [90 95 95 155]);
add_scope_block([subsys '/Speed Scope'], [180 25 240 65]);
add_scope_block([subsys '/Current Scope'], [180 100 240 140]);
add_scope_block([subsys '/Torque Scope'], [180 175 240 215]);

add_line(subsys, 'omega_m/1', 'Speed Scope/1');
add_line(subsys, 'id_meas/1', 'CurrentMux/1');
add_line(subsys, 'iq_meas/1', 'CurrentMux/2');
add_line(subsys, 'CurrentMux/1', 'Current Scope/1');
add_line(subsys, 'torque_e/1', 'Torque Scope/1');
end