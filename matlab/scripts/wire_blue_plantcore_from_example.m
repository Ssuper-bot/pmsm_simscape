function wire_blue_plantcore_from_example(wrapper_model_name, example_model_path)
%WIRE_BLUE_PLANTCORE_FROM_EXAMPLE Wire PlantCore using PMSMDrive example blocks.
%   wire_blue_plantcore_from_example
%
% This script replaces PlantCore zero-output wiring with an example-based
% physical chain and routes key measurements to wrapper outputs.

if nargin < 1 || isempty(wrapper_model_name)
    wrapper_model_name = 'my_blue_plant_wrapper';
end
if nargin < 2 || isempty(example_model_path)
    example_model_path = '/Users/wangguandi/Documents/MATLAB/Examples/R2024b/simscapeelectrical/PMSMDriveExample/PMSMDrive.slx';
end

script_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(script_dir, '..', '..');
wrapper_path = fullfile(project_root, 'matlab', 'models', [wrapper_model_name '.slx']);

if ~exist(wrapper_path, 'file')
    error('Wrapper model not found: %s', wrapper_path);
end
if ~exist(example_model_path, 'file')
    error('Example model not found: %s', example_model_path);
end

if bdIsLoaded(wrapper_model_name)
    close_system(wrapper_model_name, 0);
end

load_system(wrapper_path);
load_system(example_model_path);
[~, ex_model, ~] = fileparts(example_model_path);
plant = [wrapper_model_name '/PlantCore'];

% Clear old stub lines into outputs.
out_names = {'ia','ib','ic','omega_m','Te','theta_m'};
for i = 1:numel(out_names)
    op = [plant '/' out_names{i}];
    lh = get_param(op, 'LineHandles');
    if isfield(lh, 'Inport') && lh.Inport ~= -1
        try
            delete_line(lh.Inport);
        catch
        end
    end
end

% Remove existing staged integrated area if present.
for n = {'Ex_Battery','Ex_ElectricalRef','Ex_MechRef','Ex_LoadInertia','Ex_SolverConfig', ...
         'Ex_Inverter','Ex_SensingCurr','Ex_PMSM','Ex_Encoder', ...
         'From_i','From_wMotor','From_trqMotor','Demux_i','ThetaInt'}
    blk = [plant '/' n{1}];
    if ~isempty(find_system(plant, 'SearchDepth', 1, 'Name', n{1}))
        try
            delete_block(blk);
        catch
        end
    end
end

% Copy key + support blocks from example.
add_block([ex_model '/Battery'], [plant '/Ex_Battery'], 'Position', [280 35 340 95]);
add_block([ex_model '/' sprintf('Electrical\nReference')], [plant '/Ex_ElectricalRef'], 'Position', [260 115 330 165]);
add_block([ex_model '/' sprintf('Mechanical\nRotational Reference1')], [plant '/Ex_MechRef'], 'Position', [840 210 920 255]);
add_block([ex_model '/' sprintf('Motor &\nload inertia')], [plant '/Ex_LoadInertia'], 'Position', [760 120 850 185]);
add_block([ex_model '/' sprintf('Solver\nConfiguration')], [plant '/Ex_SolverConfig'], 'Position', [345 20 420 75]);

add_block([ex_model '/' sprintf('Three-phase\ninverter')], [plant '/Ex_Inverter'], 'Position', [420 20 560 160]);
add_block([ex_model '/Sensing currents'], [plant '/Ex_SensingCurr'], 'Position', [590 55 690 150]);
add_block([ex_model '/' sprintf('Permanent Magnet\nSynchronous Motor')], [plant '/Ex_PMSM'], 'Position', [710 40 830 170]);
add_block([ex_model '/Encoder'], [plant '/Ex_Encoder'], 'Position', [860 40 970 180]);

% Route control input gates_6 -> inverter G.
add_line(plant, 'gates_6/1', 'Ex_Inverter/1', 'autorouting', 'on');

% Physical wiring copied from PMSMDrive topology.
ph_bat = get_param([plant '/Ex_Battery'], 'PortHandles');
ph_inv = get_param([plant '/Ex_Inverter'], 'PortHandles');
ph_sens = get_param([plant '/Ex_SensingCurr'], 'PortHandles');
ph_pmsm = get_param([plant '/Ex_PMSM'], 'PortHandles');
ph_enc = get_param([plant '/Ex_Encoder'], 'PortHandles');
ph_load = get_param([plant '/Ex_LoadInertia'], 'PortHandles');
ph_mref = get_param([plant '/Ex_MechRef'], 'PortHandles');

add_line(plant, ph_bat.RConn(1), ph_inv.LConn(1), 'autorouting', 'on');
add_line(plant, ph_bat.LConn(1), ph_inv.LConn(2), 'autorouting', 'on');

add_line(plant, ph_inv.RConn(1), ph_sens.LConn(1), 'autorouting', 'on');
add_line(plant, ph_sens.RConn(1), ph_pmsm.LConn(1), 'autorouting', 'on');
add_line(plant, ph_pmsm.RConn(1), ph_enc.LConn(1), 'autorouting', 'on');
add_line(plant, ph_pmsm.RConn(2), ph_enc.LConn(2), 'autorouting', 'on');
add_line(plant, ph_enc.RConn(1), ph_load.LConn(1), 'autorouting', 'on');
add_line(plant, ph_enc.RConn(2), ph_mref.LConn(1), 'autorouting', 'on');

% Measurement extraction using global tags from copied subsystems.
add_block('simulink/Signal Routing/From', [plant '/From_i'], ...
    'GotoTag', 'i', 'Position', [995 40 1065 60]);
add_block('simulink/Signal Routing/Demux', [plant '/Demux_i'], ...
    'Outputs', '3', 'Position', [1090 28 1095 88]);
add_line(plant, 'From_i/1', 'Demux_i/1', 'autorouting', 'on');
add_line(plant, 'Demux_i/1', 'ia/1', 'autorouting', 'on');
add_line(plant, 'Demux_i/2', 'ib/1', 'autorouting', 'on');
add_line(plant, 'Demux_i/3', 'ic/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/From', [plant '/From_wMotor'], ...
    'GotoTag', 'wMotor', 'Position', [995 108 1065 128]);
add_line(plant, 'From_wMotor/1', 'omega_m/1', 'autorouting', 'on');

add_block('simulink/Continuous/Integrator', [plant '/ThetaInt'], ...
    'InitialCondition', '0', 'Position', [1095 108 1130 128]);
add_line(plant, 'From_wMotor/1', 'ThetaInt/1', 'autorouting', 'on');
add_line(plant, 'ThetaInt/1', 'theta_m/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/From', [plant '/From_trqMotor'], ...
    'GotoTag', 'trqMotor', 'Position', [995 165 1065 185]);
add_line(plant, 'From_trqMotor/1', 'Te/1', 'autorouting', 'on');

% Keep Te_load accepted by interface; currently not injected into example chain.
% This is acceptable for first bring-up and can be refined with torque source.

save_system(wrapper_model_name, wrapper_path);
open_system(wrapper_model_name);
open_system(plant);
fprintf('PlantCore wired with example-based chain in model: %s\n', wrapper_model_name);
end
