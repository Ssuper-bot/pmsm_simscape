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
addpath(simscape_dir);

testCase.TestData.SimscapeDir = simscape_dir;
end

function teardown(testCase)
close_if_loaded('pmsm_module_motor_test');
close_if_loaded('pmsm_foc_model_test');
cleanup_model_artifact('pmsm_module_motor_test', testCase.TestData.SimscapeDir);
cleanup_model_artifact('pmsm_foc_model_test', testCase.TestData.SimscapeDir);
end

function testStandaloneMotorModuleUsesSimscapePMSMAndMoves(testCase)
motor_model = 'pmsm_module_motor_test';
create_motor_module(motor_model);

assert_simscape_pmsm_present(testCase, motor_model);

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
verifyGreaterThan(testCase, max(abs(omega)), 1e-3, ...
    'Expected non-zero shaft motion in the standalone motor module.');
fprintf('  Motor module structure+motion: PASS\n');
end

function testAllInModelIncludesMotorSubsystemWithSimscapePMSM(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
all_in_model = 'pmsm_foc_model_test';
create_pmsm_foc_all_in_model(all_in_model, motor_params, inv_params, ctrl_params, sim_params, ref_params);

assert_simscape_pmsm_present(testCase, [all_in_model '/Motor']);
update_and_smoke_sim(all_in_model, min(sim_params.validation_t_end, 1e-3));
fprintf('  All-in model integration: PASS\n');
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
