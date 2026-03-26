function out_model_path = create_pmsmdrive_with_our_controller()
%CREATE_PMSMDRIVE_WITH_OUR_CONTROLLER
% Copy MathWorks PMSMDrive example model and replace controller with
% project C++ FOC S-Function + gate driver chain.

project_root = '/Users/wangguandi/Documents/pmsm_simscape';
example_model_path = '/Users/wangguandi/Documents/MATLAB/Examples/R2025b/simscapeelectrical/PMSMDriveExample/PMSMDrive.slx';
out_model_path = fullfile(project_root, 'matlab', 'models', 'PMSMDrive_with_our_controller.slx');

if ~exist(example_model_path, 'file')
    error('Example model not found: %s', example_model_path);
end

if ~exist(fileparts(out_model_path), 'dir')
    mkdir(fileparts(out_model_path));
end
copyfile(example_model_path, out_model_path);

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
cd(project_root);
addpath(project_root);
setup_project;

% Build C++ FOC S-Function MEX.
build_sfun_foc(fullfile(project_root, 'matlab', 's_function'));
if exist('sfun_foc_controller', 'file') ~= 3
    error('sfun_foc_controller MEX not found after build.');
end

load_system(out_model_path);
[~, model_name, ~] = fileparts(out_model_path);

ctrl = find_system(model_name, 'SearchDepth', 1, 'Name', 'PMSM controller');
if isempty(ctrl)
    error('Cannot find top-level PMSM controller block in %s', model_name);
end
ctrl = ctrl{1};

% Keep interface blocks only.
keepers = {'meas', 'rpm', 'i_abc', 'G'};
all_blocks = find_system(ctrl, 'SearchDepth', 1, 'Type', 'Block');
for i = 1:numel(all_blocks)
    b = all_blocks{i};
    if strcmp(b, ctrl)
        continue;
    end
    nm = get_param(b, 'Name');
    if ~any(strcmp(nm, keepers))
        delete_block(b);
    end
end

% Clear remaining lines.
lines = get_param(ctrl, 'Lines');
for i = 1:numel(lines)
    if lines(i).Handle ~= -1
        try
            delete_line(lines(i).Handle);
        catch
        end
    end
end

% Recreate our control chain.
add_block('simulink/Signal Routing/Demux', [ctrl '/Demux_meas'], ...
    'Outputs', '2', 'Position', [85 35 90 95]);
add_block('simulink/Signal Routing/Demux', [ctrl '/Demux_iabc'], ...
    'Outputs', '3', 'Position', [85 145 90 235]);

add_block('simulink/Math Operations/Gain', [ctrl '/rpm_to_radps'], ...
    'Gain', '2*pi/60', 'Position', [95 275 165 305]);
add_block('simulink/Math Operations/Gain', [ctrl '/pole_pairs'], ...
    'Gain', '4', 'Position', [145 75 205 105]);
add_block('simulink/Math Operations/Bias', [ctrl '/theta_offset'], ...
    'Bias', num2str(-pi/6, '%.12g'), 'Position', [225 75 275 105]);
add_block('built-in/Constant', [ctrl '/id_ref'], ...
    'Value', '0', 'Position', [220 315 260 335]);

add_block('simulink/Signal Routing/Mux', [ctrl '/Mux_foc_in'], ...
    'Inputs', '7', 'Position', [300 110 305 290]);

add_block('simulink/User-Defined Functions/S-Function', [ctrl '/FOC_SFun'], ...
    'FunctionName', 'sfun_foc_controller', ...
    'Parameters', ['1/20000,48,5,1000,5,1000,0.5,10,' ...
                   '0.5,1.4e-3,1.4e-3,0.0577,4,10,10'], ...
    'Position', [340 145 470 245]);

add_block('simulink/Signal Routing/Demux', [ctrl '/Demux_foc_out'], ...
    'Outputs', '5', 'Position', [500 130 505 255]);

% Duty to gate pulses
add_block('simulink/Sources/Repeating Sequence', [ctrl '/Carrier'], ...
    'rep_seq_t', '[0 5e-5]', ...
    'rep_seq_y', '[0 1]', ...
    'Position', [535 285 625 315]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [ctrl '/CmpA'], ...
    'Operator', '>=', 'Position', [560 120 610 145]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [ctrl '/CmpB'], ...
    'Operator', '>=', 'Position', [560 165 610 190]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [ctrl '/CmpC'], ...
    'Operator', '>=', 'Position', [560 210 610 235]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [ctrl '/NotA'], ...
    'Operator', 'NOT', 'Position', [640 120 675 145]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [ctrl '/NotB'], ...
    'Operator', 'NOT', 'Position', [640 165 675 190]);
add_block('simulink/Logic and Bit Operations/Logical Operator', [ctrl '/NotC'], ...
    'Operator', 'NOT', 'Position', [640 210 675 235]);

add_block('simulink/Signal Routing/Mux', [ctrl '/Mux_gates_raw'], ...
    'Inputs', '6', 'Position', [705 120 710 245]);
add_block('simulink/Signal Attributes/Data Type Conversion', [ctrl '/GateToDouble'], ...
    'OutDataTypeStr', 'double', ...
    'Position', [760 165 830 200]);

% Wiring
add_line(ctrl, 'meas/1', 'Demux_meas/1', 'autorouting', 'on');
add_line(ctrl, 'i_abc/1', 'Demux_iabc/1', 'autorouting', 'on');
add_line(ctrl, 'rpm/1', 'rpm_to_radps/1', 'autorouting', 'on');

add_line(ctrl, 'Demux_meas/1', 'Mux_foc_in/5', 'autorouting', 'on');   % omega_m
add_line(ctrl, 'Demux_meas/2', 'pole_pairs/1', 'autorouting', 'on');
add_line(ctrl, 'pole_pairs/1', 'theta_offset/1', 'autorouting', 'on');
add_line(ctrl, 'theta_offset/1', 'Mux_foc_in/4', 'autorouting', 'on'); % theta_e

add_line(ctrl, 'Demux_iabc/1', 'Mux_foc_in/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/2', 'Mux_foc_in/2', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/3', 'Mux_foc_in/3', 'autorouting', 'on');
add_line(ctrl, 'rpm_to_radps/1', 'Mux_foc_in/6', 'autorouting', 'on');
add_line(ctrl, 'id_ref/1', 'Mux_foc_in/7', 'autorouting', 'on');

add_line(ctrl, 'Mux_foc_in/1', 'FOC_SFun/1', 'autorouting', 'on');
add_line(ctrl, 'FOC_SFun/1', 'Demux_foc_out/1', 'autorouting', 'on');

add_line(ctrl, 'Demux_foc_out/1', 'CmpA/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/2', 'CmpB/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/3', 'CmpC/1', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpA/2', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpB/2', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpC/2', 'autorouting', 'on');
add_line(ctrl, 'CmpA/1', 'NotA/1', 'autorouting', 'on');
add_line(ctrl, 'CmpB/1', 'NotB/1', 'autorouting', 'on');
add_line(ctrl, 'CmpC/1', 'NotC/1', 'autorouting', 'on');

% Apply best gate order [1 3 5 2 4 6] directly in mux wiring.
add_line(ctrl, 'CmpA/1', 'Mux_gates_raw/1', 'autorouting', 'on');
add_line(ctrl, 'CmpB/1', 'Mux_gates_raw/2', 'autorouting', 'on');
add_line(ctrl, 'CmpC/1', 'Mux_gates_raw/3', 'autorouting', 'on');
add_line(ctrl, 'NotA/1', 'Mux_gates_raw/4', 'autorouting', 'on');
add_line(ctrl, 'NotB/1', 'Mux_gates_raw/5', 'autorouting', 'on');
add_line(ctrl, 'NotC/1', 'Mux_gates_raw/6', 'autorouting', 'on');
add_line(ctrl, 'Mux_gates_raw/1', 'GateToDouble/1', 'autorouting', 'on');
add_line(ctrl, 'GateToDouble/1', 'G/1', 'autorouting', 'on');

save_system(model_name, out_model_path);
close_system(model_name);
fprintf('Created model: %s\n', out_model_path);
end
