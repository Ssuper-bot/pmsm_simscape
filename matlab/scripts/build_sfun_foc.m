function build_sfun_foc(src_path)
%BUILD_SFUN_FOC Build the C++ S-Function MEX for FOC controller
%
%   build_sfun_foc(src_path)
%
%   Compiles the C++ S-Function (sfun_foc_controller.cpp) into a MEX file
%   that can be used as an S-Function block in Simulink/Simscape models.
%
%   The C++ implementation provides:
%   - Clarke/Park transforms
%   - PI current/speed controllers
%   - SVPWM modulation
%   - Decoupling feedforward
%
%   Input:
%       src_path - Path to directory containing sfun_foc_controller.cpp

    if nargin < 1
        src_path = fullfile(fileparts(mfilename('fullpath')), '..', 's_function');
    end
    
    sfun_src = fullfile(src_path, 'sfun_foc_controller.cpp');
    if ~exist(sfun_src, 'file')
        error('S-Function source not found: %s', sfun_src);
    end
    
    % C++ core algorithm sources
    cpp_core_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'cpp');
    cpp_include = fullfile(cpp_core_dir, 'include');
    cpp_src = fullfile(cpp_core_dir, 'src');
    
    % Source files to compile
    src_files = {
        sfun_src, ...
        fullfile(cpp_src, 'foc_controller.cpp'), ...
        fullfile(cpp_src, 'transforms.cpp'), ...
        fullfile(cpp_src, 'pid_controller.cpp'), ...
        fullfile(cpp_src, 'svpwm.cpp')
    };
    
    % Filter to only existing files
    existing_files = {};
    for i = 1:length(src_files)
        if exist(src_files{i}, 'file')
            existing_files{end+1} = src_files{i}; %#ok<AGROW>
        end
    end
    
    % MEX compile options
    mex_opts = {
        '-R2018a', ...                          % Modern C API
        ['-I' cpp_include], ...                 % Include path
        '-DMATLAB_MEX_FILE', ...                % MEX define
        'CXXFLAGS=$CXXFLAGS -std=c++17', ...    % C++17
    };
    
    % Output directory
    out_dir = src_path;
    
    fprintf('Compiling S-Function MEX:\n');
    for i = 1:length(existing_files)
        fprintf('  Source: %s\n', existing_files{i});
    end
    fprintf('  Include: %s\n', cpp_include);
    fprintf('  Output: %s\n', out_dir);
    
    % Compile
    mex(mex_opts{:}, '-outdir', out_dir, existing_files{:});
    
    fprintf('MEX compilation successful.\n');
    
    % Add to path
    addpath(out_dir);
    fprintf('Added %s to MATLAB path.\n', out_dir);
end
