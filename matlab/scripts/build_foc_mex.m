function build_foc_mex(out_dir)
%BUILD_FOC_MEX Build the regular C++ MEX wrapper for standalone MATLAB simulation.
%
%   build_foc_mex(out_dir)
%
%   Compiles cpp/mex/foc_controller_mex.cpp together with the C++ core
%   sources so MATLAB scripts can use the same controller implementation as
%   the Simulink S-Function.

    if nargin < 1 || isempty(out_dir)
        out_dir = fileparts(mfilename('fullpath'));
    end

    script_dir = fileparts(mfilename('fullpath'));
    project_root = fullfile(script_dir, '..', '..');
    mex_src = fullfile(project_root, 'cpp', 'mex', 'foc_controller_mex.cpp');
    cpp_core_dir = fullfile(project_root, 'cpp');
    cpp_include = fullfile(cpp_core_dir, 'include');
    cpp_src = fullfile(cpp_core_dir, 'src');

    src_files = {
        mex_src, ...
        fullfile(cpp_src, 'foc_controller.cpp'), ...
        fullfile(cpp_src, 'transforms.cpp'), ...
        fullfile(cpp_src, 'pid_controller.cpp'), ...
        fullfile(cpp_src, 'svpwm.cpp') ...
    };

    for index = 1:numel(src_files)
        if ~exist(src_files{index}, 'file')
            error('Required MEX source not found: %s', src_files{index});
        end
    end

    mex_opts = {
        '-R2018a', ...
        ['-I' cpp_include], ...
        '-DMATLAB_MEX_FILE', ...
        'CXXFLAGS=$CXXFLAGS -std=c++17' ...
    };

    fprintf('Compiling controller MEX:\n');
    fprintf('  Source: %s\n', mex_src);
    fprintf('  Include: %s\n', cpp_include);
    fprintf('  Output: %s\n', out_dir);

    mex(mex_opts{:}, '-outdir', out_dir, src_files{:});
    addpath(out_dir);
    rehash path;
    fprintf('MEX compilation successful.\n');
end