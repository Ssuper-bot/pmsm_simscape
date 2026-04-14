function build_sfun_foc(src_path)
%BUILD_SFUN_FOC Build the C++ S-Function MEX files for split FOC loops
%
%   build_sfun_foc(src_path)
%
%   Compiles:
%   - sfun_speed_controller.cpp (speed loop)
%   - sfun_foc_controller.cpp   (current loop + modulation)
%   into MEX files that can be used in Simulink/Simscape models.
%
%   The C++ implementation provides:
%   - Dedicated speed-loop S-Function
%   - Dedicated current-loop S-Function
%   - Shared C++ control core and modulation logic
%
%   Input:
%       src_path - Path to directory containing S-Function source files

    if nargin < 1
        src_path = fullfile(fileparts(mfilename('fullpath')), '..', 's_function');
    end
    
    speed_sfun_src = fullfile(src_path, 'sfun_speed_controller.cpp');
    current_sfun_src = fullfile(src_path, 'sfun_foc_controller.cpp');

    if ~exist(speed_sfun_src, 'file')
        error('Speed-loop S-Function source not found: %s', speed_sfun_src);
    end
    if ~exist(current_sfun_src, 'file')
        error('Current-loop S-Function source not found: %s', current_sfun_src);
    end
    
    % C++ core algorithm sources
    cpp_core_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'cpp');
    cpp_include = fullfile(cpp_core_dir, 'include');
    cpp_src = fullfile(cpp_core_dir, 'src');
    
    common_core_files = {
        fullfile(cpp_src, 'foc_controller.cpp'), ...
        fullfile(cpp_src, 'transforms.cpp'), ...
        fullfile(cpp_src, 'pid_controller.cpp'), ...
        fullfile(cpp_src, 'svpwm.cpp')
    };

    targets = {
        struct('name', 'sfun_speed_controller', 'src', speed_sfun_src), ...
        struct('name', 'sfun_foc_controller', 'src', current_sfun_src)
    };
    
    % MEX compile options
    mex_opts = {
        '-R2018a', ...                          % Modern C API
        ['-I' cpp_include], ...                 % Include path
        '-DMATLAB_MEX_FILE', ...                % MEX define
        'CXXFLAGS=$CXXFLAGS -std=c++17', ...    % C++17
    };
    
    % Output directory
    out_dir = src_path;

    for t = 1:numel(targets)
        target = targets{t};
        src_files = [{target.src}, common_core_files];

        existing_files = {};
        for i = 1:length(src_files)
            if exist(src_files{i}, 'file')
                existing_files{end+1} = src_files{i}; %#ok<AGROW>
            end
        end

        fprintf('Compiling %s MEX:\n', target.name);
        for i = 1:length(existing_files)
            fprintf('  Source: %s\n', existing_files{i});
        end
        fprintf('  Include: %s\n', cpp_include);
        fprintf('  Output: %s\n', out_dir);

        mex(mex_opts{:}, '-output', target.name, '-outdir', out_dir, existing_files{:});

        mex_out = fullfile(out_dir, [target.name '.' mexext]);
        if ~exist(mex_out, 'file')
            error('MEX compilation did not produce expected output: %s', mex_out);
        end
    end

    fprintf('S-Function MEX compilation successful (speed + current).\n');
    
    % Add to path
    addpath(out_dir);
    fprintf('Added %s to MATLAB path.\n', out_dir);
end
