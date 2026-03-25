function model_name = create_blue_plant_wrapper_template(model_name, open_model, plant_input_mode)
%CREATE_BLUE_PLANT_WRAPPER_TEMPLATE Create a Simscape blue-plant wrapper model template.
%
% Interface required by pmsm_foc_simscape (plant_mode=simscape_blue):
%   Inputs:  plant_cmd [3x1 duty_abc or 6x1 gates_6], Te_load [1x1]
%   Outputs: ia, ib, ic, omega_m, Te, theta_m
%
% Usage:
%   create_blue_plant_wrapper_template
%   create_blue_plant_wrapper_template('my_blue_plant_wrapper')
%   create_blue_plant_wrapper_template('my_blue_plant_wrapper', true)
%   create_blue_plant_wrapper_template('my_blue_plant_wrapper', true, 'gates_6')

    if nargin < 1 || isempty(model_name)
        model_name = 'pmsm_blue_plant_wrapper';
    end
    if nargin < 2
        open_model = true;
    end
    if nargin < 3 || isempty(plant_input_mode)
        plant_input_mode = 'gates_6';
    end

    if strcmpi(plant_input_mode, 'gates_6')
        plant_input_name = 'gates_6';
        plant_input_dim = '6';
    elseif strcmpi(plant_input_mode, 'duty_abc')
        plant_input_name = 'duty_abc';
        plant_input_dim = '3';
    else
        error('Unsupported plant_input_mode: %s. Use duty_abc or gates_6.', plant_input_mode);
    end

    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end

    model_path = fullfile(model_dir, [model_name '.slx']);
    if exist(model_path, 'file')
        delete(model_path);
    end

    new_system(model_name);

    set_param(model_name, ...
        'Solver', 'ode23t', ...
        'StopTime', '0.5', ...
        'MaxStep', '1e-5');

    % Top-level signal interface expected by Model Reference adapter.
    add_block('built-in/Inport', [model_name '/' plant_input_name], ...
        'Port', '1', ...
        'PortDimensions', plant_input_dim, ...
        'Position', [40 80 70 94]);
    add_block('built-in/Inport', [model_name '/Te_load'], ...
        'Port', '2', ...
        'Position', [40 130 70 144]);

    add_block('built-in/Outport', [model_name '/ia'], ...
        'Port', '1', ...
        'Position', [590 40 620 54]);
    add_block('built-in/Outport', [model_name '/ib'], ...
        'Port', '2', ...
        'Position', [590 80 620 94]);
    add_block('built-in/Outport', [model_name '/ic'], ...
        'Port', '3', ...
        'Position', [590 120 620 134]);
    add_block('built-in/Outport', [model_name '/omega_m'], ...
        'Port', '4', ...
        'Position', [590 160 620 174]);
    add_block('built-in/Outport', [model_name '/Te'], ...
        'Port', '5', ...
        'Position', [590 200 620 214]);
    add_block('built-in/Outport', [model_name '/theta_m'], ...
        'Port', '6', ...
        'Position', [590 240 620 254]);

    add_block('built-in/Subsystem', [model_name '/PlantCore'], ...
        'Position', [170 30 500 280]);

    create_plant_core_stub([model_name '/PlantCore'], plant_input_mode);

    add_line(model_name, [plant_input_name '/1'], 'PlantCore/1');
    add_line(model_name, 'Te_load/1', 'PlantCore/2');
    add_line(model_name, 'PlantCore/1', 'ia/1');
    add_line(model_name, 'PlantCore/2', 'ib/1');
    add_line(model_name, 'PlantCore/3', 'ic/1');
    add_line(model_name, 'PlantCore/4', 'omega_m/1');
    add_line(model_name, 'PlantCore/5', 'Te/1');
    add_line(model_name, 'PlantCore/6', 'theta_m/1');

    try
        ann = Simulink.Annotation([model_name '/TODO: Replace PlantCore stub with closed-source Simscape blue-library wiring.']);
        ann.Position = [40 300 760 330];
    catch
    end

    save_system(model_name, model_path);
    fprintf('Blue plant wrapper template saved to: %s\n', model_path);

    if open_model
        open_system(model_name);
        open_system([model_name '/PlantCore']);
    end
end

function create_plant_core_stub(subsys, plant_input_mode)
%CREATE_PLANT_CORE_STUB Compile-safe placeholder for interface bring-up.
%
% Replace this content with your real blue-library PMSM/inverter model.

    if strcmpi(plant_input_mode, 'gates_6')
        plant_input_name = 'gates_6';
        plant_input_dim = '6';
    else
        plant_input_name = 'duty_abc';
        plant_input_dim = '3';
    end

    add_block('built-in/Inport', [subsys '/' plant_input_name], 'Port', '1', 'Position', [30 50 60 64]);
    set_param([subsys '/' plant_input_name], 'PortDimensions', plant_input_dim);
    add_block('built-in/Inport', [subsys '/Te_load'],  'Port', '2', 'Position', [30 100 60 114]);

    add_block('built-in/Outport', [subsys '/ia'],      'Port', '1', 'Position', [320 40 350 54]);
    add_block('built-in/Outport', [subsys '/ib'],      'Port', '2', 'Position', [320 70 350 84]);
    add_block('built-in/Outport', [subsys '/ic'],      'Port', '3', 'Position', [320 100 350 114]);
    add_block('built-in/Outport', [subsys '/omega_m'], 'Port', '4', 'Position', [320 130 350 144]);
    add_block('built-in/Outport', [subsys '/Te'],      'Port', '5', 'Position', [320 160 350 174]);
    add_block('built-in/Outport', [subsys '/theta_m'], 'Port', '6', 'Position', [320 190 350 204]);

    add_block('simulink/Sinks/Terminator', [subsys '/Term_cmd'], ...
        'Position', [100 50 120 70]);
    add_block('simulink/Sinks/Terminator', [subsys '/Term_load'], ...
        'Position', [100 100 120 120]);

    add_block('built-in/Constant', [subsys '/Zero_ia'],      'Value', '0', 'Position', [180 35 220 55]);
    add_block('built-in/Constant', [subsys '/Zero_ib'],      'Value', '0', 'Position', [180 65 220 85]);
    add_block('built-in/Constant', [subsys '/Zero_ic'],      'Value', '0', 'Position', [180 95 220 115]);
    add_block('built-in/Constant', [subsys '/Zero_omega_m'], 'Value', '0', 'Position', [180 125 220 145]);
    add_block('built-in/Constant', [subsys '/Zero_Te'],      'Value', '0', 'Position', [180 155 220 175]);
    add_block('built-in/Constant', [subsys '/Zero_theta_m'], 'Value', '0', 'Position', [180 185 220 205]);

    add_line(subsys, [plant_input_name '/1'], 'Term_cmd/1');
    add_line(subsys, 'Te_load/1', 'Term_load/1');

    add_line(subsys, 'Zero_ia/1', 'ia/1');
    add_line(subsys, 'Zero_ib/1', 'ib/1');
    add_line(subsys, 'Zero_ic/1', 'ic/1');
    add_line(subsys, 'Zero_omega_m/1', 'omega_m/1');
    add_line(subsys, 'Zero_Te/1', 'Te/1');
    add_line(subsys, 'Zero_theta_m/1', 'theta_m/1');

    try
        ann = Simulink.Annotation([subsys '/TODO: Replace constants with signals from your real blue plant model.']);
        ann.Position = [20 235 360 260];
    catch
    end
end
