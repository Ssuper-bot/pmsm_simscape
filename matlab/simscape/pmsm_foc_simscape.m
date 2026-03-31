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

%% Default parameters
[motor_params, inv_params, ctrl_params, sim_params, ref_params] = pmsm_foc_builder('default_parameters');

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
        warning(ME.identifier, '%s', sprintf('Failed to build C++ S-Function: %s\nFalling back to MATLAB controller.', ME.message));
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
create_pmsm_foc_all_in_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

fprintf('Setup complete. Parameters loaded into workspace.\n');
fprintf('Motor: Rs=%.3f Ohm, Ld=%.3e H, Lq=%.3e H, flux=%.4f Wb, %d pole pairs\n', ...
    motor_params.Rs, motor_params.Ld, motor_params.Lq, motor_params.flux_pm, motor_params.p);
fprintf('Inverter: Vdc=%d V, fsw=%d kHz\n', inv_params.Vdc, inv_params.fsw/1e3);
fprintf('Reference: speed=%d RPM, load=%.2f N*m\n', ref_params.speed_ref, ref_params.load_torque);
fprintf('Modular entrypoints: create_*_module + validate_pmsm_foc_modules\n');
