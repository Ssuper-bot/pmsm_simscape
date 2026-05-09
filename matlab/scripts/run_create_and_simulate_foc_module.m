% run_create_and_simulate_foc_module.m
% Wrapper to create a FOC controller module and run a short validation simulation.
root = '/Users/wangguandi/Documents/pmsm_simscape';
log = fullfile(root, 'matlab', 'scripts', 'create_foc_module_run.log');
errfile = fullfile(root, 'matlab', 'scripts', 'create_foc_module_err.txt');
try
    if exist(log, 'file'), delete(log); end
    if exist(errfile, 'file'), delete(errfile); end
    diary(log);
    disp(['MATLAB starting: ' datestr(now)]);

    addpath(genpath(fullfile(root, 'matlab')));
    addpath(fullfile(root, 'matlab', 's_function'));
    addpath(genpath(fullfile(root, 'cpp')));

    % Model name
    model_name = 'pmsm_module_foc_controller_generated';

    disp(['Creating module: ' model_name]);
    model_path = pmsm_foc_builder('create_module_model', 'foc_controller', model_name);
    disp(['Model saved: ' model_path]);

    disp('Validating model (compile + short sim)');
    result = pmsm_foc_builder('validate_model', model_name, 0.02);
    disp('Validation result:');
    disp(result);

    diary('off');
catch e
    diary('off');
    fid = fopen(errfile, 'w');
    if fid > 0
        fprintf(fid, '%s\n', getReport(e, 'extended'));
        fclose(fid);
    end
    disp('ERROR_WRITTEN');
    exit(1);
end
exit(0);
