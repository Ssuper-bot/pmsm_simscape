function tests = test_simscape_mixed_rate_runtime_params
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
fprintf('Running mixed-rate/runtime-params Simscape tests...\n');

if ~exist('new_system', 'builtin') && ~exist('new_system', 'file')
    error('Simulink is required for test_simscape_mixed_rate_runtime_params.');
end

script_dir = fileparts(mfilename('fullpath'));
simscape_dir = fullfile(script_dir, '..', 'simscape');
addpath(simscape_dir);

testCase.TestData.SimscapeDir = simscape_dir;
testCase.TestData.Models = {
    'pmsm_foc_runtime_ports_test'
    'pmsm_foc_rate_split_test'
    'pmsm_module_signal_in_switch_test'
};
end

function teardown(testCase)
for i = 1:numel(testCase.TestData.Models)
    model_name = testCase.TestData.Models{i};
    close_if_loaded(model_name);
    cleanup_model_artifact(model_name, testCase.TestData.SimscapeDir);
end
end

function testAllInModelHasRuntimeParamPortsAndZOH(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
model_name = testCase.TestData.Models{1};
create_pmsm_foc_all_in_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);
load_system(model_name);

foc_ports = get_param([model_name '/FOC Controller'], 'PortHandles');
verifyEqual(testCase, numel(foc_ports.Inport), 17, 'FOC Controller should expose 17 input ports.');
verifyEqual(testCase, numel(foc_ports.Outport), 3, 'FOC Controller should expose 3 output ports.');

required_blocks = {
    'KpSpeedRt'
    'KiSpeedRt'
    'IqMaxSpeedRt'
    'KpIdRt'
    'KiIdRt'
    'KpIqRt'
    'KiIqRt'
    'IdMaxRt'
    'IqMaxRt'
    'ZOH_ia'
    'ZOH_ib'
    'ZOH_ic'
    'ZOH_theta_e'
    'ZOH_omega_m'
};
for idx = 1:numel(required_blocks)
    block_path = [model_name '/' required_blocks{idx}];
    verifyGreaterThan(testCase, getSimulinkBlockHandle(block_path), 0, ...
        sprintf('Expected block missing: %s', block_path));
end

zoh_ts = str2double(get_param([model_name '/ZOH_ia'], 'SampleTime'));
verifyEqual(testCase, zoh_ts, sim_params.Ts_control, 'AbsTol', 1e-12, ...
    'Measurement ZOH sample time should match Ts_control.');
end

function testSplitSampleTimesPropagateToSFunctions(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');

sim_params.Ts_control = 1 / inv_params.fsw;
sim_params.Ts_speed = 10 * sim_params.Ts_control;
model_name = testCase.TestData.Models{2};
create_pmsm_foc_all_in_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);
load_system(model_name);

speed_params = parse_param_vector(get_param([model_name '/FOC Controller/Speed Core'], 'Parameters'));
foc_params = parse_param_vector(get_param([model_name '/FOC Controller/FOC Core'], 'Parameters'));

verifyGreaterThanOrEqual(testCase, numel(speed_params), 1);
verifyGreaterThanOrEqual(testCase, numel(foc_params), 1);
verifyEqual(testCase, speed_params(1), sim_params.Ts_speed, 'AbsTol', 1e-12, ...
    'Speed Core sample time should use Ts_speed.');
verifyEqual(testCase, foc_params(1), sim_params.Ts_control, 'AbsTol', 1e-12, ...
    'FOC Core sample time should use Ts_control.');
end

function testSignalInSupportsExternalInputSwitch(testCase)
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');
ref_params.use_external_inputs = true;

model_name = testCase.TestData.Models{3};
pmsm_foc_builder('create_module_model', 'signal_in', model_name, ...
    motor_params, inv_params, ctrl_params, sim_params, ref_params);
load_system(model_name);

signal_ports = get_param([model_name '/Signal In'], 'PortHandles');
verifyEqual(testCase, numel(signal_ports.Inport), 4, 'Signal In should expose 4 external command inputs.');
verifyEqual(testCase, numel(signal_ports.Outport), 4, 'Signal In should expose 4 output signals.');

use_external = str2double(get_param([model_name '/Signal In/UseExternalInputs'], 'Value'));
verifyEqual(testCase, use_external, 1, 'use_external_inputs=true should set UseExternalInputs to 1.');

required_switches = {
    'Speed Select'
    'Id Select'
    'Load Select'
    'Throttle Select'
};
for idx = 1:numel(required_switches)
    block_path = [model_name '/Signal In/' required_switches{idx}];
    verifyGreaterThan(testCase, getSimulinkBlockHandle(block_path), 0, ...
        sprintf('Expected selector missing: %s', block_path));
end
end

function values = parse_param_vector(param_text)
parts = regexp(param_text, '\s*,\s*', 'split');
values = nan(1, numel(parts));
for idx = 1:numel(parts)
    values(idx) = str2double(parts{idx});
end
values = values(~isnan(values));
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
