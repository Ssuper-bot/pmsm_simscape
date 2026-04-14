function project_root = setup_project(action)
%SETUP_PROJECT Initialize MATLAB paths and expose common project entrypoints.
%   SETUP_PROJECT adds the project's MATLAB folders to the active MATLAB
%   path, prints the main commands, and optionally launches a post-setup
%   utility.
%
%   Usage (in MATLAB command window):
%     >> cd('/Users/wangguandi/Documents/pmsm_simscape')
%     >> setup_project
%     >> setup_project('compare')
%
%   Optional actions:
%     'compare'   Launch compare_first_order_bandwidths UI after setup.

if nargin < 1
	action = '';
end

project_root = fileparts(mfilename('fullpath'));
path_specs = {
	fullfile(project_root, 'matlab'), 'MATLAB package root';
	fullfile(project_root, 'matlab', 'scripts'), 'MATLAB scripts';
	fullfile(project_root, 'matlab', 'simscape'), 'Simscape builder scripts';
	fullfile(project_root, 'matlab', 's_function'), 'S-Function sources and MEX';
	fullfile(project_root, 'matlab', 'tests'), 'MATLAB tests';
};

fprintf('=== PMSM FOC Project ===\n');
fprintf('Project root: %s\n', project_root);

added_count = 0;
for idx = 1:size(path_specs, 1)
	folder_path = path_specs{idx, 1};
	if exist(folder_path, 'dir')
		addpath(folder_path);
		added_count = added_count + 1;
	else
		warning('setup_project:missingFolder', ...
			'Skipped missing folder: %s (%s)', folder_path, path_specs{idx, 2});
	end
end
rehash path;

fprintf('MATLAB paths refreshed. Added %d project folders.\n\n', added_count);
fprintf('Available commands:\n');
fprintf('  run_pmsm_simulation           - Standalone FOC simulation (no Simscape needed)\n');
fprintf('  compare_first_order_bandwidths - Compare PI zero cancellation, mismatch, and P-only cases\n');
fprintf('  pmsm_foc_simscape             - Simscape model setup (requires Simulink + Simscape)\n');
fprintf('  create_*_module               - Build standalone Simulink module harness models\n');
fprintf('  create_pmsm_foc_all_in_model  - Build the assembled all-in Simulink model\n');
fprintf('  validate_pmsm_foc_modules     - Build/compile/short-sim all modules and all-in\n');
fprintf('  test_pmsm_lib                 - Run MATLAB unit tests\n');
fprintf('  build_sfun_foc                - Compile C++ S-Function MEX\n');
fprintf('  build_foc_mex                 - Compile standalone C++ controller MEX\n');

normalized_action = lower(string(action));
switch normalized_action
	case ""
	case {"compare", "compare_first_order_bandwidths"}
		fprintf('\nLaunching compare_first_order_bandwidths ...\n');
		compare_first_order_bandwidths();
	otherwise
		error('setup_project:unknownAction', ...
			'Unknown action "%s". Supported actions: compare', action);
end
end
