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
ctrl_params.Kp_id = 5.0;
ctrl_params.Ki_id = 1000.0;
ctrl_params.Kp_iq = 5.0;
ctrl_params.Ki_iq = 1000.0;
ctrl_params.Kp_speed = 0.5;
ctrl_params.Ki_speed = 10.0;
ctrl_params.id_max = 10.0;
ctrl_params.iq_max = 10.0;
ctrl_params.speed_max = 3000;

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
end

function model_path = create_module_model(module_name, model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
short_stop_time = min(sim_params.validation_t_end, sim_params.t_end);
model_path = initialize_model(model_name, sim_params, short_stop_time);

switch char(module_name)
    case 'signal_in'
        add_block('built-in/Subsystem', [model_name '/Signal In']);
        create_signal_in_subsystem([model_name '/Signal In'], ref_params);
        set_param([model_name '/Signal In'], 'Position', [120 80 300 240]);

        add_scope_block([model_name '/Speed Ref Scope'], [430 80 500 120]);
        add_scope_block([model_name '/Id Ref Scope'], [430 145 500 185]);
        add_scope_block([model_name '/Load Scope'], [430 210 500 250]);

        add_line(model_name, 'Signal In/1', 'Speed Ref Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Signal In/2', 'Id Ref Scope/1', 'autorouting', 'smart');
        add_line(model_name, 'Signal In/3', 'Load Scope/1', 'autorouting', 'smart');

    case 'foc_controller'
        add_block('built-in/Subsystem', [model_name '/FOC Controller']);
        create_foc_controller_subsystem([model_name '/FOC Controller'], ctrl_params, inv_params);
        set_param([model_name '/FOC Controller'], 'Position', [280 80 560 390]);

        add_constant_source(model_name, 'Ia', '0.0', [40 60 90 80]);
        add_constant_source(model_name, 'Ib', '-1.0', [40 105 90 125]);
        add_constant_source(model_name, 'Ic', '1.0', [40 150 90 170]);
        add_ramp_source(model_name, 'ThetaE', '2*pi*50', [30 195 100 225]);
        add_constant_source(model_name, 'OmegaM', '0.0', [40 245 90 265]);
        add_step_source(model_name, 'SpeedRef', '0.001', '0', num2str(ref_params.speed_ref), [25 290 100 320]);
        add_constant_source(model_name, 'IdRef', num2str(ref_params.id_ref), [40 345 90 365]);

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
model_path = initialize_model(model_name, sim_params, sim_params.t_end);

add_block('built-in/Subsystem', [model_name '/Signal In']);
create_signal_in_subsystem([model_name '/Signal In'], ref_params);
set_param([model_name '/Signal In'], 'Position', [60 90 240 250]);

add_block('built-in/Subsystem', [model_name '/FOC Controller']);
create_foc_controller_subsystem([model_name '/FOC Controller'], ctrl_params, inv_params);
set_param([model_name '/FOC Controller'], 'Position', [300 80 560 390]);

add_block('built-in/Subsystem', [model_name '/Three Invertor']);
create_power_stage([model_name '/Three Invertor'], inv_params);
set_param([model_name '/Three Invertor'], 'Position', [640 145 820 315]);

add_block('built-in/Subsystem', [model_name '/Motor']);
create_pmsm_subsystem([model_name '/Motor'], motor_params);
set_param([model_name '/Motor'], 'Position', [920 100 1160 380]);

add_block('built-in/Subsystem', [model_name '/Measure']);
create_measurement_subsystem([model_name '/Measure'], motor_params);
set_param([model_name '/Measure'], 'Position', [930 440 1160 650]);

add_block('built-in/Subsystem', [model_name '/Scope']);
create_scope_subsystem([model_name '/Scope']);
set_param([model_name '/Scope'], 'Position', [1270 150 1470 360]);

add_line(model_name, 'Signal In/1', 'FOC Controller/6', 'autorouting', 'smart');
add_line(model_name, 'Signal In/2', 'FOC Controller/7', 'autorouting', 'smart');
add_line(model_name, 'Signal In/3', 'Motor/4', 'autorouting', 'smart');

add_line(model_name, 'FOC Controller/1', 'Three Invertor/1', 'autorouting', 'smart');

add_line(model_name, 'Three Invertor/1', 'Motor/1', 'autorouting', 'smart');
add_line(model_name, 'Three Invertor/2', 'Motor/2', 'autorouting', 'smart');
add_line(model_name, 'Three Invertor/3', 'Motor/3', 'autorouting', 'smart');

add_line(model_name, 'Motor/1', 'Measure/1', 'autorouting', 'smart');
add_line(model_name, 'Motor/2', 'Measure/2', 'autorouting', 'smart');
add_line(model_name, 'Motor/3', 'Measure/3', 'autorouting', 'smart');
add_line(model_name, 'Motor/6', 'Measure/4', 'autorouting', 'smart');
add_line(model_name, 'Motor/4', 'Measure/5', 'autorouting', 'smart');

add_line(model_name, 'Measure/1', 'FOC Controller/1', 'autorouting', 'smart');
add_line(model_name, 'Measure/2', 'FOC Controller/2', 'autorouting', 'smart');
add_line(model_name, 'Measure/3', 'FOC Controller/3', 'autorouting', 'smart');
add_line(model_name, 'Measure/4', 'FOC Controller/4', 'autorouting', 'smart');
add_line(model_name, 'Measure/5', 'FOC Controller/5', 'autorouting', 'smart');

add_line(model_name, 'Motor/4', 'Scope/1', 'autorouting', 'smart');
add_line(model_name, 'FOC Controller/2', 'Scope/2', 'autorouting', 'smart');
add_line(model_name, 'FOC Controller/3', 'Scope/3', 'autorouting', 'smart');
add_line(model_name, 'Motor/5', 'Scope/4', 'autorouting', 'smart');

save_generated_model(model_name, model_path);
end

function result = validate_model(model_name, stop_time)
result = struct('model_name', model_name, 'saved', false, 'compiled', false, ...
    'simulated', false, 'message', '');

model_path = get_model_path(model_name);
result.saved = exist(model_path, 'file') == 2;
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

model_path = get_model_path(model_name);
if exist(model_path, 'file') == 2
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
save_system(model_name, model_path);
fprintf('Model saved to: %s\n', model_path);
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

add_block('simulink/Math Operations/Product', [subsys '/Modulator'], ...
    'Inputs', '2', ...
    'Multiplication', 'Element-wise(.*)', ...
    'Position', [200 80 250 130]);

add_block('simulink/Signal Routing/Demux', [subsys '/Demux'], ...
    'Outputs', '3', ...
    'Position', [300 70 310 150]);

add_line(subsys, 'duty_abc/1', 'Modulator/1');
add_line(subsys, 'Vdc/1', 'Modulator/2');
add_line(subsys, 'Modulator/1', 'Demux/1');
add_line(subsys, 'Demux/1', 'Va/1');
add_line(subsys, 'Demux/2', 'Vb/1');
add_line(subsys, 'Demux/3', 'Vc/1');
end

function create_foc_controller_subsystem(subsys, ctrl_params, inv_params)
add_block('built-in/Inport', [subsys '/ia'], 'Port', '1');
add_block('built-in/Inport', [subsys '/ib'], 'Port', '2');
add_block('built-in/Inport', [subsys '/ic'], 'Port', '3');
add_block('built-in/Inport', [subsys '/theta_e'], 'Port', '4');
add_block('built-in/Inport', [subsys '/omega_m'], 'Port', '5');
add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6');
add_block('built-in/Inport', [subsys '/id_ref'], 'Port', '7');

add_block('built-in/Outport', [subsys '/duty_abc'], 'Port', '1');
add_block('built-in/Outport', [subsys '/id_meas'], 'Port', '2');
add_block('built-in/Outport', [subsys '/iq_meas'], 'Port', '3');

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
add_line(subsys, 'Demux_dq/1', 'id_meas/1');
add_line(subsys, 'Demux_dq/2', 'iq_meas/1');

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

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_svpwm'], ...
    'Inputs', '3', 'Position', [1030 200 1035 270]);
add_block('built-in/Constant', [subsys '/Vdc_const'], ...
    'Value', num2str(inv_params.Vdc), ...
    'Position', [980 270 1020 290]);
add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/SVPWM'], ...
    'MATLABFcn', 'svpwm_inline(u(1), u(2), u(3))', ...
    'Position', [1070 210 1180 250]);

add_line(subsys, 'Demux_vab/1', 'Mux_svpwm/1');
add_line(subsys, 'Demux_vab/2', 'Mux_svpwm/2');
add_line(subsys, 'Vdc_const/1', 'Mux_svpwm/3');
add_line(subsys, 'Mux_svpwm/1', 'SVPWM/1');
add_line(subsys, 'SVPWM/1', 'duty_abc/1');
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

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_in'], ...
    'Inputs', '4', 'Position', [100 80 105 230]);
add_line(subsys, 'Va/1', 'Mux_in/1');
add_line(subsys, 'Vb/1', 'Mux_in/2');
add_line(subsys, 'Vc/1', 'Mux_in/3');
add_line(subsys, 'Te_load/1', 'Mux_in/4');

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_all'], ...
    'Inputs', '2', 'Position', [200 100 205 200]);

    Rs_s = num2str(motor_params.Rs, '%.6g');
    Ld_s = num2str(motor_params.Ld, '%.6g');
    Lq_s = num2str(motor_params.Lq, '%.6g');
    fp_s = num2str(motor_params.flux_pm, '%.6g');
    pp_s = num2str(motor_params.p);
    J_s = num2str(motor_params.J, '%.6g');
    B_s = num2str(motor_params.B, '%.6g');

    fcn_str = ['pmsm_model_fcn(u(1),u(2),u(3),u(4),u(5),u(6),u(7),u(8),' ...
        Rs_s ',' Ld_s ',' Lq_s ',' fp_s ',' pp_s ',' J_s ',' B_s ')'];

add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/PMSM_Eqn'], ...
    'MATLABFcn', fcn_str, ...
    'Position', [250 120 420 180]);
add_block('simulink/Signal Routing/Demux', [subsys '/Demux_out'], ...
    'Outputs', '8', 'Position', [460 80 465 260]);

add_line(subsys, 'Mux_in/1', 'Mux_all/1');
add_line(subsys, 'Mux_all/1', 'PMSM_Eqn/1');
add_line(subsys, 'PMSM_Eqn/1', 'Demux_out/1');

states = {'id', 'iq', 'wm', 'theta'};
y_pos = [80, 120, 160, 200];
for state_index = 1:4
    blk = [subsys '/Int_' states{state_index}];
    add_block('simulink/Continuous/Integrator', blk, ...
        'Position', [530 y_pos(state_index) 570 y_pos(state_index) + 20]);
    add_line(subsys, ['Demux_out/' num2str(state_index)], ['Int_' states{state_index} '/1']);
end

add_block('simulink/Signal Routing/Mux', [subsys '/Mux_states'], ...
    'Inputs', '4', 'Position', [620 80 625 220]);
for state_index = 1:4
    add_line(subsys, ['Int_' states{state_index} '/1'], ['Mux_states/' num2str(state_index)]);
end
add_line(subsys, 'Mux_states/1', 'Mux_all/2');

add_line(subsys, 'Demux_out/5', 'ia/1');
add_line(subsys, 'Demux_out/6', 'ib/1');
add_line(subsys, 'Demux_out/7', 'ic/1');
add_line(subsys, 'Int_wm/1', 'omega_m/1');
add_line(subsys, 'Demux_out/8', 'Te/1');
add_line(subsys, 'Int_theta/1', 'theta_m/1');
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

function create_signal_in_subsystem(subsys, ref_params)
add_block('built-in/Outport', [subsys '/speed_ref'], 'Port', '1');
add_block('built-in/Outport', [subsys '/id_ref'], 'Port', '2');
add_block('built-in/Outport', [subsys '/Te_load'], 'Port', '3');

add_block('simulink/Sources/Ramp', [subsys '/Speed Ramp'], ...
    'Slope', num2str(ref_params.speed_ref / ref_params.speed_ramp_time), ...
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

add_line(subsys, 'Speed Ramp/1', 'Speed Sat/1');
add_line(subsys, 'Speed Sat/1', 'speed_ref/1');
add_line(subsys, 'Id Ref/1', 'id_ref/1');
add_line(subsys, 'Load Step/1', 'Te_load/1');
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