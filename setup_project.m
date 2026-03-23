%% PMSM FOC Project Setup
% Run this script ONCE to add all project paths to MATLAB.
% After running this, you can call any function/script from anywhere.
%
% Usage (in MATLAB command window):
%   >> cd('/Users/wangguandi/Documents/pmsm_simscape')
%   >> setup_project
%
% Then you can run:
%   >> run_pmsm_simulation        % Standalone simulation (no Simscape needed)
%   >> pmsm_foc_simscape          % Simscape model setup (needs Simulink)

project_root = fileparts(mfilename('fullpath'));

% Add all MATLAB subdirectories to path
addpath(fullfile(project_root, 'matlab'));                % +pmsm_lib package
addpath(fullfile(project_root, 'matlab', 'scripts'));     % utility scripts
addpath(fullfile(project_root, 'matlab', 'simscape'));    % Simscape scripts
addpath(fullfile(project_root, 'matlab', 's_function')); % S-Function
addpath(fullfile(project_root, 'matlab', 'tests'));       % tests

fprintf('=== PMSM FOC Project ===\n');
fprintf('Project root: %s\n', project_root);
fprintf('MATLAB paths added.\n\n');
fprintf('Available commands:\n');
fprintf('  run_pmsm_simulation   - Standalone FOC simulation (no Simscape needed)\n');
fprintf('  pmsm_foc_simscape     - Simscape model setup (requires Simulink + Simscape)\n');
fprintf('  test_pmsm_lib         - Run MATLAB unit tests\n');
fprintf('  build_sfun_foc        - Compile C++ S-Function MEX\n');
