function [model_name, sim_out] = pmsm_foc_simscape(overrides)
%PMSM_FOC_SIMSCAPE Build and run PMSM FOC model using C++ S-Function controller.
%
% Usage:
%   pmsm_foc_simscape
%   pmsm_foc_simscape(struct('run_sim', false))
%
% Note:
%   This function enforces the C++ S-Function controller path and fails fast
%   if the MEX build cannot be completed.

if nargin < 1
    overrides = struct();
end

sim_out = [];

if ~exist('bdIsLoaded', 'file') && ~exist('new_system', 'file')
    error(['pmsm_foc_simscape requires Simulink.\n' ...
           'If you don''t have Simulink, use run_pmsm_simulation instead.\n' ...
           'Usage:\n  >> run_pmsm_simulation']);
end

script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..');
addpath(matlab_dir);
addpath(fullfile(matlab_dir, 'scripts'));
addpath(script_dir);
rehash path;

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
sim_params.controller_mode = 'sfun';
sim_params.current_noise_std = 0.02;
sim_params.theta_noise_std = 1e-3;
sim_params.noise_seed = 42;
sim_params.run_sim = true;

ref_params = struct();
ref_params.speed_ref = 1000;
ref_params.speed_ramp_time = 0.1;
ref_params.load_torque = 0.1;
ref_params.load_step_time = 0.3;
ref_params.id_ref = 0;

if isfield(overrides, 'motor_params')
    motor_params = merge_struct(motor_params, overrides.motor_params);
end
if isfield(overrides, 'inv_params')
    inv_params = merge_struct(inv_params, overrides.inv_params);
end
if isfield(overrides, 'ctrl_params')
    ctrl_params = merge_struct(ctrl_params, overrides.ctrl_params);
end
if isfield(overrides, 'sim_params')
    sim_params = merge_struct(sim_params, overrides.sim_params);
end
if isfield(overrides, 'ref_params')
    ref_params = merge_struct(ref_params, overrides.ref_params);
end

if isfield(overrides, 'run_sim')
    sim_params.run_sim = logical(overrides.run_sim);
end

assignin('base', 'motor_params', motor_params);
assignin('base', 'inv_params', inv_params);
assignin('base', 'ctrl_params', ctrl_params);
assignin('base', 'sim_params', sim_params);
assignin('base', 'ref_params', ref_params);

cpp_sfun_path = fullfile(fileparts(mfilename('fullpath')), '..', 's_function');
if ~exist(fullfile(cpp_sfun_path, 'sfun_foc_controller.cpp'), 'file')
    error('S-Function source is missing at %s', cpp_sfun_path);
end

fprintf('Building C++ S-Function MEX...\n');
build_sfun_foc(cpp_sfun_path);

if exist('sfun_foc_controller', 'file') ~= 3
    error('sfun_foc_controller MEX is not available on MATLAB path after build.');
end

model_name = 'pmsm_foc_model';
model_path = fullfile(fileparts(mfilename('fullpath')), '..', 'models', [model_name '.slx']);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if exist(model_path, 'file')
    delete(model_path);
end

fprintf('Creating model programmatically...\n');
create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

fprintf('Model setup complete.\n');
fprintf('Motor: Rs=%.3f Ohm, Ld=%.3e H, Lq=%.3e H, flux=%.4f Wb, %d pole pairs\n', ...
    motor_params.Rs, motor_params.Ld, motor_params.Lq, motor_params.flux_pm, motor_params.p);
fprintf('Inverter: Vdc=%.1f V, fsw=%.1f kHz\n', inv_params.Vdc, inv_params.fsw/1e3);
fprintf('Scenario: speed step to %.1f RPM, load step %.2f N*m @ %.3f s\n', ...
    ref_params.speed_ref, ref_params.load_torque, ref_params.load_step_time);
fprintf('Noise: current std=%.4f A, theta std=%.4g rad\n', ...
    sim_params.current_noise_std, sim_params.theta_noise_std);

if sim_params.run_sim
    fprintf('Running simulation to %.3f s...\n', sim_params.t_end);
    sim_out = sim(model_name, 'StopTime', num2str(sim_params.t_end));
    fprintf('Simulation finished successfully.\n');
    open_system(model_name);
end
end

function dst = merge_struct(dst, src)
fields = fieldnames(src);
for i = 1:numel(fields)
    name = fields{i};
    if isfield(dst, name) && isstruct(dst.(name)) && isstruct(src.(name)) ...
            && isscalar(dst.(name)) && isscalar(src.(name))
        % Recursively merge nested scalar structs
        dst.(name) = merge_struct(dst.(name), src.(name));
    else
        % Default behavior: shallow overwrite
        dst.(name) = src.(name);
    end
end
end
