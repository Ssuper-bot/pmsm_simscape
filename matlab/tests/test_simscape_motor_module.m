function tests = test_simscape_motor_module
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
fprintf('Running Simscape motor module tests...\n');

if ~exist('new_system', 'builtin') && ~exist('new_system', 'file')
    error('Simulink is required for test_simscape_motor_module.');
end

script_dir = fileparts(mfilename('fullpath'));
simscape_dir = fullfile(script_dir, '..', 'simscape');
scripts_dir = fullfile(script_dir, '..', 'scripts');
sfun_dir = fullfile(script_dir, '..', 's_function');
addpath(simscape_dir);
addpath(scripts_dir);
addpath(sfun_dir);

testCase.TestData.SimscapeDir = simscape_dir;
testCase.TestData.SFunDir = sfun_dir;
end

function teardown(testCase)
close_if_loaded('pmsm_module_motor_test');
close_if_loaded('pmsm_module_three_invertor_test');
close_if_loaded('pmsm_foc_model_test');
cleanup_model_artifact('pmsm_module_motor_test', testCase.TestData.SimscapeDir);
cleanup_model_artifact('pmsm_module_three_invertor_test', testCase.TestData.SimscapeDir);
cleanup_model_artifact('pmsm_foc_model_test', testCase.TestData.SimscapeDir);
end

function testStandaloneMotorModuleUsesSimscapePMSMAndSimulates(testCase)
motor_model = 'pmsm_module_motor_test';
create_motor_module(motor_model);

assert_simscape_pmsm_present(testCase, motor_model);
assert_switching_inverter_present(testCase, [motor_model '/Three Invertor']);

load_system(motor_model);
set_param(motor_model, 'SimulationCommand', 'update');
motor_ports = get_param([motor_model '/Motor'], 'PortHandles');
set_param(motor_ports.Outport(4), ...
    'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', 'omega_m');
set_param(motor_model, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

sim_out = sim(motor_model, 'StopTime', '0.05', 'ReturnWorkspaceOutputs', 'on');
omega = extract_logged_signal(sim_out.get('logsout'), 'omega_m');

verifyNotEmpty(testCase, omega, 'omega_m signal was not logged.');
verifyTrue(testCase, all(isfinite(omega(:))), ...
    'Expected finite shaft speed samples from the standalone motor module.');
fprintf('  Motor module structure+smoke: PASS\n');
end

function testStandaloneInverterModuleSwitches(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
inv_params.dead_time = 5e-6;
sim_params.max_step = min(sim_params.max_step, inv_params.dead_time / 20);

inv_model = 'pmsm_module_three_invertor_test';
pmsm_foc_builder('create_module_model', 'three_invertor', inv_model, ...
    motor_params, inv_params, ctrl_params, sim_params, ref_params);

assert_switching_inverter_present(testCase, [inv_model '/Three Invertor']);

load_system(inv_model);
set_param(inv_model, 'SimulationCommand', 'update');
gate_h_ports = get_param([inv_model '/Three Invertor/GateA_H_Drive'], 'PortHandles');
gate_l_ports = get_param([inv_model '/Three Invertor/GateA_L_Drive'], 'PortHandles');
set_param(gate_h_ports.Outport(1), ...
    'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', 'gate_a_h');
set_param(gate_l_ports.Outport(1), ...
    'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', ...
    'DataLoggingName', 'gate_a_l');
set_param(inv_model, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

sim_out = sim(inv_model, 'StopTime', '0.002', 'ReturnWorkspaceOutputs', 'on');
gate_a_h = extract_logged_signal(sim_out.get('logsout'), 'gate_a_h');
gate_a_l = extract_logged_signal(sim_out.get('logsout'), 'gate_a_l');
gate_sum = gate_a_h + gate_a_l;

verifyNotEmpty(testCase, gate_a_h, 'High-side inverter gate signal was not logged.');
verifyNotEmpty(testCase, gate_a_l, 'Low-side inverter gate signal was not logged.');
verifyLessThan(testCase, min(gate_a_h(:)), 0.5, ...
    'Expected the logged high-side gate command to reach the low state.');
verifyGreaterThan(testCase, max(gate_a_h(:)), 0.5 * inv_params.gate_drive_voltage, ...
    'Expected the logged high-side gate command to reach the high state.');
verifyLessThan(testCase, min(gate_a_l(:)), 0.5, ...
    'Expected the logged low-side gate command to reach the low state.');
verifyGreaterThan(testCase, max(gate_a_l(:)), 0.5 * inv_params.gate_drive_voltage, ...
    'Expected the logged low-side gate command to reach the high state.');
verifyLessThanOrEqual(testCase, max(gate_sum(:)), inv_params.gate_drive_voltage + 1e-9, ...
    'Expected dead-time gating to prevent simultaneous high-side and low-side turn-on.');
verifyGreaterThan(testCase, nnz(gate_sum(:) < 1.0), 0, ...
    'Expected at least one sampled dead-time interval where both phase-leg MOSFETs are off.');
fprintf('  Inverter module MOSFET + dead-time smoke: PASS\n');
end

function testAllInModelIncludesMotorSubsystemWithSimscapePMSM(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
all_in_model = 'pmsm_foc_model_test';
create_pmsm_foc_all_in_model(all_in_model, motor_params, inv_params, ctrl_params, sim_params, ref_params);

assert_simscape_pmsm_present(testCase, [all_in_model '/Motor']);
assert_switching_inverter_present(testCase, [all_in_model '/Three Invertor']);
ensure_required_sfun_built(testCase.TestData.SFunDir);
update_and_smoke_sim(all_in_model, min(sim_params.validation_t_end, 1e-3));
fprintf('  All-in model integration: PASS\n');
end

function ensure_required_sfun_built(sfun_dir)
speed_mex = fullfile(sfun_dir, ['sfun_speed_controller.' mexext]);
current_mex = fullfile(sfun_dir, ['sfun_foc_controller.' mexext]);
speed_src = fullfile(sfun_dir, 'sfun_speed_controller.cpp');
current_src = fullfile(sfun_dir, 'sfun_foc_controller.cpp');

need_build = ~isfile(speed_mex) || ~isfile(current_mex);
if ~need_build
    speed_src_info = dir(speed_src);
    current_src_info = dir(current_src);
    speed_mex_info = dir(speed_mex);
    current_mex_info = dir(current_mex);

    need_build = speed_src_info.datenum > speed_mex_info.datenum || ...
        current_src_info.datenum > current_mex_info.datenum;
end

if ~need_build
    return;
end

build_sfun_foc(sfun_dir);
end

function assert_simscape_pmsm_present(testCase, root_path)
blocks = find_system(root_path, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'FindAll', 'off', ...
    'Regexp', 'on', ...
    'ReferenceBlock', 'ee_lib/Electromechanical/Permanent Magnet/PMSM');
verifyNotEmpty(testCase, blocks, sprintf('Expected Simscape PMSM block was not found in %s', root_path));

required_refs = {
    'ee_lib/Sensors & Transducers/Current Sensor \(Three-Phase\)'
    'fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor'
    'fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor'
};
for idx = 1:numel(required_refs)
    ref = required_refs{idx};
    found = find_system(root_path, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'FindAll', 'off', ...
        'Regexp', 'on', ...
        'ReferenceBlock', ref);
    verifyNotEmpty(testCase, found, sprintf('Missing expected Simscape support block: %s', ref));
end
end

function assert_switching_inverter_present(testCase, root_path)
mosfets = find_system(root_path, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'FindAll', 'off', ...
    'ReferenceBlock', 'ee_lib/Semiconductors & Converters/MOSFET (Ideal, Switching)');
verifyEqual(testCase, numel(mosfets), 6, ...
    sprintf('Expected 6 ideal-switching MOSFET blocks in %s.', root_path));

phase_splitter = find_system(root_path, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'FindAll', 'off', ...
    'ReferenceBlock', 'ee_lib/Connectors & References/Phase Splitter');
verifyNotEmpty(testCase, phase_splitter, ...
    sprintf('Expected a Phase Splitter block in %s.', root_path));
end

function update_and_smoke_sim(model_name, stop_time)
load_system(model_name);
set_param(model_name, 'SimulationCommand', 'update');
sim(model_name, 'StopTime', num2str(stop_time));
close_if_loaded(model_name);
end

function values = extract_logged_signal(logs, signal_name)
values = [];
if isempty(logs)
    return;
end

for idx = 1:logs.numElements
    element = logs{idx};
    if strcmp(element.Name, signal_name)
        values = element.Values.Data;
        return;
    end
end
end

function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end

function cleanup_model_artifact(model_name, simscape_dir)
model_path = fullfile(simscape_dir, '..', 'models', [model_name '.slx']);
if isfile(model_path)
    delete(model_path);
end
end
