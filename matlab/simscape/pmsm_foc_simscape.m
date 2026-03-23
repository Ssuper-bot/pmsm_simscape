%% PMSM FOC Simscape Simulation - Main Setup Script
% This script creates and configures the PMSM Field-Oriented Control
% simulation model using Simscape Electrical components.
%
% Requires: Simulink, Simscape, Simscape Electrical
%
% Usage (in MATLAB command window):
%   >> cd('/path/to/pmsm_simscape')
%   >> setup_project          % Add paths (run once)
%   >> pmsm_foc_simscape      % Run this script
%
% If you don't have Simulink, use:
%   >> run_pmsm_simulation    % Standalone simulation

%% Check Simulink availability
if ~exist('bdIsLoaded', 'file') && ~exist('new_system', 'file')
    error(['pmsm_foc_simscape requires Simulink.\n' ...
           'If you don''t have Simulink, use run_pmsm_simulation instead.\n' ...
           'Usage:\n  >> run_pmsm_simulation']);
end

%% Add paths if not already set
script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..');
addpath(matlab_dir);             % +pmsm_lib package
addpath(fullfile(matlab_dir, 'scripts'));  % build_sfun_foc etc.
addpath(script_dir);             % create_pmsm_foc_model etc.
rehash path;                     % Force MATLAB to re-scan path

%% Clear workspace
clear motor_params inv_params ctrl_params sim_params ref_params;

%% Motor Parameters (typical PMSM)
motor_params = struct();
motor_params.Rs = 0.5;           % Stator resistance [Ohm]
motor_params.Ld = 1.4e-3;        % d-axis inductance [H]
motor_params.Lq = 1.4e-3;        % q-axis inductance [H]
motor_params.flux_pm = 0.0577;   % Permanent magnet flux linkage [Wb]
motor_params.p = 4;              % Number of pole pairs
motor_params.J = 1.74e-5;        % Rotor inertia [kg*m^2]
motor_params.B = 1e-4;           % Viscous damping [N*m*s]

%% Inverter Parameters
inv_params = struct();
inv_params.Vdc = 48;             % DC bus voltage [V]
inv_params.fsw = 20e3;           % Switching frequency [Hz]
inv_params.Tsw = 1/inv_params.fsw;
inv_params.dead_time = 1e-6;     % Dead time [s]

%% Controller Parameters
ctrl_params = struct();
% Current loop PI gains (d-axis)
ctrl_params.Kp_id = 5.0;
ctrl_params.Ki_id = 1000.0;
% Current loop PI gains (q-axis)
ctrl_params.Kp_iq = 5.0;
ctrl_params.Ki_iq = 1000.0;
% Speed loop PI gains
ctrl_params.Kp_speed = 0.5;
ctrl_params.Ki_speed = 10.0;
% Limits
ctrl_params.id_max = 10.0;      % Max d-axis current [A]
ctrl_params.iq_max = 10.0;      % Max q-axis current [A]
ctrl_params.speed_max = 3000;   % Max speed [RPM]

%% Simulation Parameters
sim_params = struct();
sim_params.Ts_control = 1/inv_params.fsw;  % Control sample time
sim_params.Ts_speed = 10 * sim_params.Ts_control;  % Speed loop sample time
sim_params.t_end = 0.5;           % Simulation end time [s]
sim_params.solver = 'ode23t';     % Solver (ode23t for Simscape)
sim_params.max_step = 1e-5;       % Max step size

%% Reference Signals
ref_params = struct();
ref_params.speed_ref = 1000;      % Reference speed [RPM]
ref_params.speed_ramp_time = 0.1; % Speed ramp time [s]
ref_params.load_torque = 0.1;     % Load torque [N*m]
ref_params.load_step_time = 0.3;  % Load step time [s]
ref_params.id_ref = 0;            % Reference d-axis current (MTPA: id=0)

%% Save parameters to workspace for Simulink model
assignin('base', 'motor_params', motor_params);
assignin('base', 'inv_params', inv_params);
assignin('base', 'ctrl_params', ctrl_params);
assignin('base', 'sim_params', sim_params);
assignin('base', 'ref_params', ref_params);

%% Build or load S-Function MEX if C++ controller is available
cpp_sfun_path = fullfile(fileparts(mfilename('fullpath')), '..', 's_function');
if exist(fullfile(cpp_sfun_path, 'sfun_foc_controller.cpp'), 'file')
    fprintf('C++ S-Function source found. Building MEX...\n');
    try
        build_sfun_foc(cpp_sfun_path);
        fprintf('S-Function MEX built successfully.\n');
    catch ME
        warning('Failed to build C++ S-Function: %s\nFalling back to MATLAB controller.', ME.message);
    end
end

%% Create the Simulink model (always regenerate to pick up latest fixes)
model_name = 'pmsm_foc_model';
model_path = fullfile(fileparts(mfilename('fullpath')), '..', 'models', [model_name '.slx']);

% Close old model if loaded
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Delete stale .slx so we always get a clean, fully-wired model
if exist(model_path, 'file')
    fprintf('Deleting old model to regenerate: %s\n', model_path);
    delete(model_path);
end

fprintf('Creating model programmatically...\n');
create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

fprintf('Setup complete. Parameters loaded into workspace.\n');
fprintf('Motor: Rs=%.3f Ohm, Ld=%.3e H, Lq=%.3e H, flux=%.4f Wb, %d pole pairs\n', ...
    motor_params.Rs, motor_params.Ld, motor_params.Lq, motor_params.flux_pm, motor_params.p);
fprintf('Inverter: Vdc=%d V, fsw=%d kHz\n', inv_params.Vdc, inv_params.fsw/1e3);
fprintf('Reference: speed=%d RPM, load=%.2f N*m\n', ref_params.speed_ref, ref_params.load_torque);
