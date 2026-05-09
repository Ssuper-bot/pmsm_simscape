% run_build_sfun_foc_wrapper.m
% Wrapper to run build_sfun_foc with full path and capture logs/errors.
root = '/Users/wangguandi/Documents/pmsm_simscape';
log = fullfile(root, 'matlab', 'scripts', 'build_sfun_foc_run.log');
errfile = fullfile(root, 'matlab', 'scripts', 'build_sfun_foc_error.txt');
try
    if exist(log, 'file'), delete(log); end
    if exist(errfile, 'file'), delete(errfile); end
    diary(log);
    disp(['MATLAB starting: ' datestr(now)]);
    disp(['MATLAB version: ' version]);

    % Ensure repo scripts and s_function folders are on path
    addpath(genpath(fullfile(root, 'matlab')));
    addpath(genpath(fullfile(root, 'matlab', 'scripts')));
    addpath(genpath(fullfile(root, 'matlab', 's_function')));
    addpath(genpath(fullfile(root, 'cpp')));

    disp('Searching for build_sfun_foc:');
    which build_sfun_foc -all

    disp('Calling build_sfun_foc...');
    build_sfun_foc(fullfile(root, 'matlab', 's_function'));

    disp('BUILD_OK');
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
