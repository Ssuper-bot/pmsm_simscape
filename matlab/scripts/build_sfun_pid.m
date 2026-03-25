function build_sfun_pid(src_path)
%BUILD_SFUN_PID Build reusable PI S-Function MEX.
%
%   build_sfun_pid(src_path)
%
%   Compiles sfun_pi_controller.cpp into a MEX file for Simulink use.

    if nargin < 1
        src_path = fullfile(fileparts(mfilename('fullpath')), '..', 's_function');
    end

    sfun_src = fullfile(src_path, 'sfun_pi_controller.cpp');
    if ~exist(sfun_src, 'file')
        error('PI S-Function source not found: %s', sfun_src);
    end

    cpp_core_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'cpp');
    cpp_include = fullfile(cpp_core_dir, 'include');
    cpp_src = fullfile(cpp_core_dir, 'src');

    src_files = {
        sfun_src, ...
        fullfile(cpp_src, 'pid_controller.cpp')
    };

    mex_opts = {
        '-R2018a', ...
        ['-I' cpp_include], ...
        '-DMATLAB_MEX_FILE', ...
        'CXXFLAGS=$CXXFLAGS -std=c++17', ...
    };

    out_dir = src_path;

    fprintf('Compiling PI S-Function MEX:\n');
    fprintf('  Source: %s\n', sfun_src);
    fprintf('  Include: %s\n', cpp_include);
    fprintf('  Output: %s\n', out_dir);

    mex(mex_opts{:}, '-outdir', out_dir, src_files{:});

    fprintf('PI S-Function MEX compilation successful.\n');
    addpath(out_dir);
end
