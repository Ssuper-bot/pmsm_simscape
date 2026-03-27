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
bdclose('all');
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

% Make short-run verification meaningful: trigger speed step inside 0.05 s window.
set_param(model_name, 'StopTime', '0.05');
set_param([model_name '/Step'], 'Time', '0.01', 'Before', '0', 'After', '500');

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
add_block('simulink/Math Operations/Gain', [ctrl '/CurrentSign'], ...
    'Gain', '1', 'Position', [95 165 155 195]);
add_block('simulink/Signal Routing/Demux', [ctrl '/Demux_iabc'], ...
    'Outputs', '3', 'Position', [85 145 90 235]);

add_block('simulink/Math Operations/Gain', [ctrl '/rpm_to_radps'], ...
    'Gain', '2*pi/60', 'Position', [95 275 165 305]);
add_block('simulink/Math Operations/Gain', [ctrl '/pole_pairs'], ...
    'Gain', '4', 'Position', [145 75 205 105]);
add_block('built-in/Constant', [ctrl '/theta_offset'], ...
    'Value', '-pi/2', 'Position', [220 85 260 105]);
add_block('simulink/Math Operations/Add', [ctrl '/theta_e_sum'], ...
    'Inputs', '++', 'Position', [280 78 300 112]);
add_block('built-in/Constant', [ctrl '/id_ref'], ...
    'Value', '0', 'Position', [220 315 260 335]);

add_block('simulink/Signal Routing/Mux', [ctrl '/Mux_foc_in'], ...
    'Inputs', '7', 'Position', [300 110 305 290]);

add_block('simulink/User-Defined Functions/S-Function', [ctrl '/FOC_SFun'], ...
    'FunctionName', 'sfun_foc_controller', ...
    'Parameters', ['1/20000,48,5,1000,5,1000,0.5,10,' ...
                   '0.5,1.4e-3,1.4e-3,0.0577,4,10,10,0'], ...
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

% Centralized signal hub for input grouping and input/output comparisons.
add_block('built-in/Subsystem', [ctrl '/signals'], ...
    'Position', [840 40 1090 360]);
create_signals_subsystem([ctrl '/signals']);

% Restore tags consumed by Signals subsystem in the example scope.
add_block('simulink/Sinks/To Workspace', [ctrl '/id_meas_ws'], ...
    'VariableName', 'id_meas_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [545 150 635 170]);
add_block('simulink/Sinks/To Workspace', [ctrl '/iq_meas_ws'], ...
    'VariableName', 'iq_meas_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [545 190 635 210]);
add_block('simulink/Sinks/To Workspace', [ctrl '/theta_e_ws'], ...
    'VariableName', 'theta_e_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [335 70 425 90]);
add_block('simulink/Sinks/To Workspace', [ctrl '/omega_m_ws'], ...
    'VariableName', 'omega_m_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [335 100 425 120]);
add_block('simulink/Sinks/To Workspace', [ctrl '/ia_ws'], ...
    'VariableName', 'ia_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [205 130 285 150]);
add_block('simulink/Sinks/To Workspace', [ctrl '/ib_ws'], ...
    'VariableName', 'ib_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [205 165 285 185]);
add_block('simulink/Sinks/To Workspace', [ctrl '/ic_ws'], ...
    'VariableName', 'ic_dbg', 'SaveFormat', 'Timeseries', ...
    'Position', [205 200 285 220]);

% Wiring
add_line(ctrl, 'meas/1', 'Demux_meas/1', 'autorouting', 'on');
add_line(ctrl, 'i_abc/1', 'CurrentSign/1', 'autorouting', 'on');
add_line(ctrl, 'CurrentSign/1', 'Demux_iabc/1', 'autorouting', 'on');
add_line(ctrl, 'rpm/1', 'rpm_to_radps/1', 'autorouting', 'on');

add_line(ctrl, 'Demux_meas/2', 'pole_pairs/1', 'autorouting', 'on');
add_line(ctrl, 'pole_pairs/1', 'theta_e_sum/1', 'autorouting', 'on');
add_line(ctrl, 'theta_offset/1', 'theta_e_sum/2', 'autorouting', 'on');
add_line(ctrl, 'theta_e_sum/1', 'theta_e_ws/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_meas/1', 'omega_m_ws/1', 'autorouting', 'on');

add_line(ctrl, 'Demux_iabc/1', 'ia_ws/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/2', 'ib_ws/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/3', 'ic_ws/1', 'autorouting', 'on');

% Route all FOC inputs through the centralized signals subsystem.
add_line(ctrl, 'Demux_iabc/1', 'signals/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/2', 'signals/2', 'autorouting', 'on');
add_line(ctrl, 'Demux_iabc/3', 'signals/3', 'autorouting', 'on');
add_line(ctrl, 'theta_e_sum/1', 'signals/4', 'autorouting', 'on');
add_line(ctrl, 'Demux_meas/1', 'signals/5', 'autorouting', 'on');
add_line(ctrl, 'rpm_to_radps/1', 'signals/6', 'autorouting', 'on');
add_line(ctrl, 'id_ref/1', 'signals/7', 'autorouting', 'on');

add_line(ctrl, 'signals/1', 'Mux_foc_in/1', 'autorouting', 'on');
add_line(ctrl, 'signals/2', 'Mux_foc_in/2', 'autorouting', 'on');
add_line(ctrl, 'signals/3', 'Mux_foc_in/3', 'autorouting', 'on');
add_line(ctrl, 'signals/4', 'Mux_foc_in/4', 'autorouting', 'on');
add_line(ctrl, 'signals/5', 'Mux_foc_in/5', 'autorouting', 'on');
add_line(ctrl, 'signals/6', 'Mux_foc_in/6', 'autorouting', 'on');
add_line(ctrl, 'signals/7', 'Mux_foc_in/7', 'autorouting', 'on');

add_line(ctrl, 'Mux_foc_in/1', 'FOC_SFun/1', 'autorouting', 'on');
add_line(ctrl, 'FOC_SFun/1', 'Demux_foc_out/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/4', 'id_meas_ws/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/5', 'iq_meas_ws/1', 'autorouting', 'on');

% Feed measured outputs and gate vector into signals subsystem for compare scopes.
add_line(ctrl, 'Demux_foc_out/4', 'signals/8', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/5', 'signals/9', 'autorouting', 'on');

add_line(ctrl, 'Demux_foc_out/1', 'CmpA/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/2', 'CmpB/1', 'autorouting', 'on');
add_line(ctrl, 'Demux_foc_out/3', 'CmpC/1', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpA/2', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpB/2', 'autorouting', 'on');
add_line(ctrl, 'Carrier/1', 'CmpC/2', 'autorouting', 'on');
add_line(ctrl, 'CmpA/1', 'NotA/1', 'autorouting', 'on');
add_line(ctrl, 'CmpB/1', 'NotB/1', 'autorouting', 'on');
add_line(ctrl, 'CmpC/1', 'NotC/1', 'autorouting', 'on');

% Apply gate order [1 2 3 4 5 6] = [Sa+ Sa- Sb+ Sb- Sc+ Sc-].
add_line(ctrl, 'CmpA/1', 'Mux_gates_raw/1', 'autorouting', 'on');
add_line(ctrl, 'NotA/1', 'Mux_gates_raw/2', 'autorouting', 'on');
add_line(ctrl, 'CmpB/1', 'Mux_gates_raw/3', 'autorouting', 'on');
add_line(ctrl, 'NotB/1', 'Mux_gates_raw/4', 'autorouting', 'on');
add_line(ctrl, 'CmpC/1', 'Mux_gates_raw/5', 'autorouting', 'on');
add_line(ctrl, 'NotC/1', 'Mux_gates_raw/6', 'autorouting', 'on');
add_line(ctrl, 'Mux_gates_raw/1', 'GateToDouble/1', 'autorouting', 'on');
add_line(ctrl, 'GateToDouble/1', 'G/1', 'autorouting', 'on');
add_line(ctrl, 'GateToDouble/1', 'signals/10', 'autorouting', 'on');

save_system(model_name, out_model_path, 'OverwriteIfChangedOnDisk', true);
close_system(model_name);
fprintf('Created model: %s\n', out_model_path);
end

function create_signals_subsystem(signals_blk)
%CREATE_SIGNALS_SUBSYSTEM Group controller inputs and wire compare scopes.
% Inports:
%   1: ia, 2: ib, 3: ic, 4: theta_e, 5: omega_m,
%   6: speed_ref, 7: id_ref, 8: id_meas, 9: iq_meas, 10: gates
% Outports:
%   1: ia, 2: ib, 3: ic, 4: theta_e, 5: omega_m, 6: speed_ref, 7: id_ref

add_block('built-in/Inport', [signals_blk '/ia'], 'Port', '1', ...
    'Position', [40 30 70 45]);
add_block('built-in/Inport', [signals_blk '/ib'], 'Port', '2', ...
    'Position', [40 60 70 75]);
add_block('built-in/Inport', [signals_blk '/ic'], 'Port', '3', ...
    'Position', [40 90 70 105]);
add_block('built-in/Inport', [signals_blk '/theta_e'], 'Port', '4', ...
    'Position', [40 120 70 135]);
add_block('built-in/Inport', [signals_blk '/omega_m'], 'Port', '5', ...
    'Position', [40 150 70 165]);
add_block('built-in/Inport', [signals_blk '/speed_ref'], 'Port', '6', ...
    'Position', [40 180 70 195]);
add_block('built-in/Inport', [signals_blk '/id_ref'], 'Port', '7', ...
    'Position', [40 210 70 225]);
add_block('built-in/Inport', [signals_blk '/id_meas'], 'Port', '8', ...
    'Position', [40 240 70 255]);
add_block('built-in/Inport', [signals_blk '/iq_meas'], 'Port', '9', ...
    'Position', [40 270 70 285]);
add_block('built-in/Inport', [signals_blk '/gates'], 'Port', '10', ...
    'Position', [40 300 70 315]);

add_block('built-in/Outport', [signals_blk '/ia_out'], 'Port', '1', ...
    'Position', [200 30 230 45]);
add_block('built-in/Outport', [signals_blk '/ib_out'], 'Port', '2', ...
    'Position', [200 60 230 75]);
add_block('built-in/Outport', [signals_blk '/ic_out'], 'Port', '3', ...
    'Position', [200 90 230 105]);
add_block('built-in/Outport', [signals_blk '/theta_e_out'], 'Port', '4', ...
    'Position', [200 120 230 135]);
add_block('built-in/Outport', [signals_blk '/omega_m_out'], 'Port', '5', ...
    'Position', [200 150 230 165]);
add_block('built-in/Outport', [signals_blk '/speed_ref_out'], 'Port', '6', ...
    'Position', [200 180 230 195]);
add_block('built-in/Outport', [signals_blk '/id_ref_out'], 'Port', '7', ...
    'Position', [200 210 230 225]);

add_line(signals_blk, 'ia/1', 'ia_out/1', 'autorouting', 'on');
add_line(signals_blk, 'ib/1', 'ib_out/1', 'autorouting', 'on');
add_line(signals_blk, 'ic/1', 'ic_out/1', 'autorouting', 'on');
add_line(signals_blk, 'theta_e/1', 'theta_e_out/1', 'autorouting', 'on');
add_line(signals_blk, 'omega_m/1', 'omega_m_out/1', 'autorouting', 'on');
add_line(signals_blk, 'speed_ref/1', 'speed_ref_out/1', 'autorouting', 'on');
add_line(signals_blk, 'id_ref/1', 'id_ref_out/1', 'autorouting', 'on');

% Legacy global tags expected by the example's Signals scopes.
add_block('simulink/Signal Routing/Mux', [signals_blk '/Mux_iMotor'], ...
    'Inputs', '3', 'Position', [270 30 275 105]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_iMotor'], ...
    'GotoTag', 'iMotor', 'TagVisibility', 'global', ...
    'Position', [300 55 385 75]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_wDemand'], ...
    'GotoTag', 'wDemand', 'TagVisibility', 'global', ...
    'Position', [300 180 390 200]);
add_block('built-in/Constant', [signals_blk '/trqDemand_zero'], ...
    'Value', '0', 'Position', [280 210 320 230]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_trqDemand'], ...
    'GotoTag', 'trqDemand', 'TagVisibility', 'global', ...
    'Position', [340 210 435 230]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_G'], ...
    'GotoTag', 'G', 'TagVisibility', 'global', ...
    'Position', [300 300 360 320]);

add_line(signals_blk, 'ia/1', 'Mux_iMotor/1', 'autorouting', 'on');
add_line(signals_blk, 'ib/1', 'Mux_iMotor/2', 'autorouting', 'on');
add_line(signals_blk, 'ic/1', 'Mux_iMotor/3', 'autorouting', 'on');
add_line(signals_blk, 'Mux_iMotor/1', 'Goto_iMotor/1', 'autorouting', 'on');
add_line(signals_blk, 'speed_ref/1', 'Goto_wDemand/1', 'autorouting', 'on');
add_line(signals_blk, 'trqDemand_zero/1', 'Goto_trqDemand/1', 'autorouting', 'on');
add_line(signals_blk, 'gates/1', 'Goto_G/1', 'autorouting', 'on');

% Compare tags for scope plotting.
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_cmp_speed_ref'], ...
    'GotoTag', 'cmp_speed_ref', 'TagVisibility', 'local', ...
    'Position', [460 180 560 200]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_cmp_omega_m'], ...
    'GotoTag', 'cmp_omega_m', 'TagVisibility', 'local', ...
    'Position', [460 150 560 170]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_cmp_id_ref'], ...
    'GotoTag', 'cmp_id_ref', 'TagVisibility', 'local', ...
    'Position', [460 210 560 230]);
add_block('simulink/Signal Routing/Goto', [signals_blk '/Goto_cmp_id_meas'], ...
    'GotoTag', 'cmp_id_meas', 'TagVisibility', 'local', ...
    'Position', [460 240 560 260]);

add_line(signals_blk, 'speed_ref/1', 'Goto_cmp_speed_ref/1', 'autorouting', 'on');
add_line(signals_blk, 'omega_m/1', 'Goto_cmp_omega_m/1', 'autorouting', 'on');
add_line(signals_blk, 'id_ref/1', 'Goto_cmp_id_ref/1', 'autorouting', 'on');
add_line(signals_blk, 'id_meas/1', 'Goto_cmp_id_meas/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/From', [signals_blk '/From_cmp_speed_ref'], ...
    'GotoTag', 'cmp_speed_ref', 'Position', [600 180 680 200]);
add_block('simulink/Signal Routing/From', [signals_blk '/From_cmp_omega_m'], ...
    'GotoTag', 'cmp_omega_m', 'Position', [600 150 680 170]);
add_block('simulink/Signal Routing/From', [signals_blk '/From_cmp_id_ref'], ...
    'GotoTag', 'cmp_id_ref', 'Position', [600 240 680 260]);
add_block('simulink/Signal Routing/From', [signals_blk '/From_cmp_id_meas'], ...
    'GotoTag', 'cmp_id_meas', 'Position', [600 270 680 290]);

add_block('simulink/Signal Routing/Mux', [signals_blk '/Mux_speed_cmp'], ...
    'Inputs', '2', 'Position', [710 155 715 205]);
add_block('simulink/Signal Routing/Mux', [signals_blk '/Mux_id_cmp'], ...
    'Inputs', '2', 'Position', [710 245 715 295]);

add_block('simulink/Sinks/Scope', [signals_blk '/Scope_speed_cmp'], ...
    'Position', [760 155 830 205]);
add_block('simulink/Sinks/Scope', [signals_blk '/Scope_id_cmp'], ...
    'Position', [760 245 830 295]);

add_line(signals_blk, 'From_cmp_omega_m/1', 'Mux_speed_cmp/1', 'autorouting', 'on');
add_line(signals_blk, 'From_cmp_speed_ref/1', 'Mux_speed_cmp/2', 'autorouting', 'on');
add_line(signals_blk, 'From_cmp_id_meas/1', 'Mux_id_cmp/1', 'autorouting', 'on');
add_line(signals_blk, 'From_cmp_id_ref/1', 'Mux_id_cmp/2', 'autorouting', 'on');
add_line(signals_blk, 'Mux_speed_cmp/1', 'Scope_speed_cmp/1', 'autorouting', 'on');
add_line(signals_blk, 'Mux_id_cmp/1', 'Scope_id_cmp/1', 'autorouting', 'on');
end
