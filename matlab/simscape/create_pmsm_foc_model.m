function create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params)
%CREATE_PMSM_FOC_MODEL Create PMSM FOC model with Simscape Electrical components
%
% Requires: Simulink, Simscape, Simscape Electrical
%
% Architecture
% ------------
% Physical domain (Simscape Electrical):
%   DC Voltage Source (+/-) --> Three-Phase Inverter (ee_lib)
%   Three-Phase Inverter (a/b/c) --> Phase Current Sensors (ee_lib) --> PMSM (ee_lib)
%   PMSM rotor (R) --> Ideal Rotational Motion Sensor + Load Torque Source --> Mech Ref
%   PMSM case  (C) --> Mechanical Reference
%   Solver Configuration attached to the electrical network
%
% Signal domain (Simulink):
%   Phase current sensors  --> ia, ib, ic
%   Rotational motion sensor --> theta_m, omega_m
%   Measurements subsystem   --> theta_e = p*theta_m  (our encoder module)
%   FOC Controller           --> G[6] gate signals --> Three-Phase Inverter
%   Reference Generator      --> speed_ref, id_ref --> FOC Controller
%
% FOC Controller subsystem (our own algorithm):
%   Inputs:  ia, ib, ic, theta_e, omega_m, speed_ref, id_ref
%   Stages:  Clarke -> Park -> Speed PI -> Current PI (d/q) -> InvPark -> SVPWM -> PWM Gen
%   Output:  G = [g1 g2 g3 g4 g5 g6]  (6 gate signals for the 6 inverter switches)
%
% Output Signals (logged):
%   Scopes: Speed, Phase Currents, dq Currents, Electromagnetic Torque
%   To Workspace: omega_m, ia/ib/ic, id/iq, Te

    %% ── Create model ─────────────────────────────────────────────────────────
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    new_system(model_name);
    open_system(model_name);

    % Simscape requires variable-step solver; ode23t is recommended for stiff
    % electrical/mechanical systems with switching.
    set_param(model_name, ...
        'Solver',   sim_params.solver, ...
        'StopTime', num2str(sim_params.t_end), ...
        'MaxStep',  num2str(sim_params.max_step), ...
        'RelTol',   '1e-4', ...
        'AbsTol',   '1e-6');

    %% ── Simscape solver configuration ────────────────────────────────────────
    % The Solver Configuration block must be present in every Simscape network.
    solver_cfg = [model_name '/Solver Configuration'];
    add_block('nesl_utility/Solver Configuration', solver_cfg, ...
        'Position', [30 30 130 70]);

    %% ── DC Bus ───────────────────────────────────────────────────────────────
    dc_src = [model_name '/DC Voltage Source'];
    add_block('fl_lib/Electrical/Electrical Sources/DC Voltage Source', dc_src, ...
        'dc_voltage', num2str(inv_params.Vdc), ...
        'Position', [200 60 260 110]);

    elec_ref = [model_name '/Electrical Reference'];
    add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference', elec_ref, ...
        'Position', [200 150 260 180]);

    %% ── Three-Phase Inverter (Simscape Electrical) ────────────────────────────
    % Try primary ee_lib path, then sps_lib fallback.
    inverter = [model_name '/Three-Phase Inverter'];
    inv_added = false;
    inv_paths = { ...
        'ee_lib/Power Electronics/Converters/Three-Phase Inverter', ...
        'sps_lib/Power Electronics/Three-Phase Inverter', ...
        'ee_lib/Power Electronics/Three-Phase Inverter'};
    for k = 1:numel(inv_paths)
        try
            add_block(inv_paths{k}, inverter, 'Position', [360 60 480 200]);
            inv_added = true;
            fprintf('Three-Phase Inverter added from: %s\n', inv_paths{k});
            break;
        catch
        end
    end
    if ~inv_added
        error(['Could not find Three-Phase Inverter block.\n' ...
               'Ensure Simscape Electrical is installed and licensed.\n' ...
               'Tried: %s'], strjoin(inv_paths, ', '));
    end

    %% ── Phase Current Sensors ────────────────────────────────────────────────
    % Three series current sensors on phases A, B, C (ee_lib).
    cs_paths = { ...
        'ee_lib/Sensors & Transducers/Current Sensor', ...
        'fl_lib/Electrical/Electrical Sensors/Current Sensor'};
    phase_names = {'A', 'B', 'C'};
    cs_blk = cell(1,3);
    for ph = 1:3
        cs_blk{ph} = [model_name '/Current Sensor ' phase_names{ph}];
        cs_added = false;
        for k = 1:numel(cs_paths)
            try
                add_block(cs_paths{k}, cs_blk{ph}, ...
                    'Position', [550 50+(ph-1)*80 630 90+(ph-1)*80]);
                cs_added = true;
                break;
            catch
            end
        end
        if ~cs_added
            error('Could not add Current Sensor block. Ensure Simscape Electrical is installed.');
        end
    end

    %% ── PMSM (Simscape Electrical) ───────────────────────────────────────────
    pmsm = [model_name '/PMSM'];
    pmsm_added = false;
    pmsm_paths = { ...
        'ee_lib/Electromechanical/Permanent Magnet/PMSM', ...
        'ee_lib/Electromechanical/Permanent Magnet/Permanent Magnet Synchronous Motor', ...
        'sps_lib/Machines/Permanent Magnet Synchronous Motor'};
    for k = 1:numel(pmsm_paths)
        try
            add_block(pmsm_paths{k}, pmsm, 'Position', [700 60 820 280]);
            pmsm_added = true;
            fprintf('PMSM block added from: %s\n', pmsm_paths{k});
            break;
        catch
        end
    end
    if ~pmsm_added
        error(['Could not find PMSM block.\n' ...
               'Ensure Simscape Electrical is installed and licensed.\n' ...
               'Tried: %s'], strjoin(pmsm_paths, ', '));
    end

    % Configure PMSM parameters.
    % Parameter names follow ee_lib PMSM block convention (R2022b+).
    % If your MATLAB version uses different names, adjust accordingly.
    try
        set_param(pmsm, ...
            'num_pole_pairs', num2str(motor_params.p), ...
            'R',              num2str(motor_params.Rs), ...
            'Ld',             num2str(motor_params.Ld), ...
            'Lq',             num2str(motor_params.Lq), ...
            'lambda',         num2str(motor_params.flux_pm), ...
            'J',              num2str(motor_params.J), ...
            'B',              num2str(motor_params.B));
    catch ME_param
        warning('Could not set all PMSM parameters automatically: %s\n%s', ...
            ME_param.identifier, ME_param.message);
    end

    %% ── Mechanical Network ───────────────────────────────────────────────────
    % Ideal Rotational Motion Sensor measures rotor angle and speed.
    rms = [model_name '/Rotational Motion Sensor'];
    add_block('foundation_lib/Mechanical/Rotational/Sensors/Ideal Rotational Motion Sensor', ...
        rms, 'Position', [900 60 980 130]);

    % Mechanical Reference (ground for the rotational network).
    mech_ref = [model_name '/Mechanical Reference'];
    add_block('foundation_lib/Mechanical/Rotational/Elements/Rotational Reference', ...
        mech_ref, 'Position', [900 330 960 370]);

    % Load Torque Source (applies a step load torque to the rotor shaft).
    torque_src = [model_name '/Load Torque Source'];
    add_block('foundation_lib/Mechanical/Rotational/Sources/Torque Source', ...
        torque_src, 'Position', [900 200 980 270]);

    % PS-Simulink converters for sensor outputs (theta, omega).
    ps2sl_theta = [model_name '/PS-Simulink theta'];
    add_block('nesl_utility/PS-Simulink Converter', ps2sl_theta, ...
        'Position', [1020 65 1090 95]);

    ps2sl_omega = [model_name '/PS-Simulink omega'];
    add_block('nesl_utility/PS-Simulink Converter', ps2sl_omega, ...
        'Position', [1020 105 1090 135]);

    % Simulink-PS converter for torque signal input.
    sl2ps_torque = [model_name '/Simulink-PS torque'];
    add_block('nesl_utility/Simulink-PS Converter', sl2ps_torque, ...
        'Position', [820 220 890 250]);

    %% ── Simscape physical network connections ─────────────────────────────────
    % DC Bus: Electrical Reference --> DC Source- --> Inverter DC-
    add_line(model_name, 'Electrical Reference/LConn1', 'DC Voltage Source/-', ...
        'autorouting', 'smart');
    add_line(model_name, 'DC Voltage Source/-', 'Three-Phase Inverter/-', ...
        'autorouting', 'smart');
    % DC+: DC Source+ --> Inverter DC+
    add_line(model_name, 'DC Voltage Source/+', 'Three-Phase Inverter/+', ...
        'autorouting', 'smart');

    % Solver Configuration attached to electrical ground node
    add_line(model_name, 'Solver Configuration/RConn1', 'Electrical Reference/LConn1', ...
        'autorouting', 'smart');

    % Inverter AC phases --> Current Sensors --> PMSM phases
    % Phase A
    add_line(model_name, 'Three-Phase Inverter/a', 'Current Sensor A/+', ...
        'autorouting', 'smart');
    add_line(model_name, 'Current Sensor A/-', 'PMSM/a', ...
        'autorouting', 'smart');
    % Phase B
    add_line(model_name, 'Three-Phase Inverter/b', 'Current Sensor B/+', ...
        'autorouting', 'smart');
    add_line(model_name, 'Current Sensor B/-', 'PMSM/b', ...
        'autorouting', 'smart');
    % Phase C
    add_line(model_name, 'Three-Phase Inverter/c', 'Current Sensor C/+', ...
        'autorouting', 'smart');
    add_line(model_name, 'Current Sensor C/-', 'PMSM/c', ...
        'autorouting', 'smart');

    % PMSM case (C port) --> Mechanical Reference
    add_line(model_name, 'PMSM/C', 'Mechanical Reference/LConn1', ...
        'autorouting', 'smart');

    % PMSM rotor (R port) --> Rotational Motion Sensor R
    add_line(model_name, 'PMSM/R', 'Rotational Motion Sensor/R', ...
        'autorouting', 'smart');
    % PMSM rotor (R port) --> Torque Source R (same shaft node -- branch)
    add_line(model_name, 'PMSM/R', 'Load Torque Source/R', ...
        'autorouting', 'smart');

    % Rotational Motion Sensor C --> Mechanical Reference
    add_line(model_name, 'Rotational Motion Sensor/C', 'Mechanical Reference/LConn1', ...
        'autorouting', 'smart');

    % Torque Source S (case) --> Mechanical Reference
    add_line(model_name, 'Load Torque Source/C', 'Mechanical Reference/LConn1', ...
        'autorouting', 'smart');

    % PS outputs from motion sensor --> PS-Simulink converters
    add_line(model_name, 'Rotational Motion Sensor/theta', 'PS-Simulink theta/LConn1', ...
        'autorouting', 'smart');
    add_line(model_name, 'Rotational Motion Sensor/w', 'PS-Simulink omega/LConn1', ...
        'autorouting', 'smart');

    % Simulink-PS converter output --> Torque Source input
    add_line(model_name, 'Simulink-PS torque/RConn1', 'Load Torque Source/t', ...
        'autorouting', 'smart');

    %% ── Simulink signal blocks ───────────────────────────────────────────────

    % --- Reference Generator ---
    ref_gen = [model_name '/Reference Generator'];
    add_block('built-in/Subsystem', ref_gen);
    create_reference_generator(ref_gen, ref_params);
    set_param(ref_gen, 'Position', [50 430 200 510]);

    % --- Mechanical Load (step torque signal) ---
    load_step = [model_name '/Load Step'];
    add_block('simulink/Sources/Step', load_step, ...
        'Time',   num2str(ref_params.load_step_time), ...
        'Before', '0', ...
        'After',  num2str(ref_params.load_torque), ...
        'Position', [640 225 700 255]);

    % --- Measurements subsystem (our own encoder module) ---
    meas_block = [model_name '/Measurements'];
    add_block('built-in/Subsystem', meas_block);
    create_measurement_subsystem(meas_block, motor_params);
    set_param(meas_block, 'Position', [1150 80 1320 240]);

    % --- FOC Controller subsystem ---
    foc_ctrl = [model_name '/FOC Controller'];
    add_block('built-in/Subsystem', foc_ctrl);
    create_foc_controller_subsystem(foc_ctrl, ctrl_params, inv_params);
    set_param(foc_ctrl, 'Position', [50 540 270 760]);

    %% ── Simulink signal connections ──────────────────────────────────────────

    % Load step --> Simulink-PS converter
    add_line(model_name, 'Load Step/1', 'Simulink-PS torque/LConn1', ...
        'autorouting', 'smart');

    % Current sensors Simulink outputs --> Measurements subsystem
    add_line(model_name, 'Current Sensor A/I', 'Measurements/1', 'autorouting', 'smart');
    add_line(model_name, 'Current Sensor B/I', 'Measurements/2', 'autorouting', 'smart');
    add_line(model_name, 'Current Sensor C/I', 'Measurements/3', 'autorouting', 'smart');

    % PS-Simulink converters --> Measurements subsystem
    add_line(model_name, 'PS-Simulink theta/1', 'Measurements/4', 'autorouting', 'smart');
    add_line(model_name, 'PS-Simulink omega/1', 'Measurements/5', 'autorouting', 'smart');

    % Measurements --> FOC Controller
    % Output port ordering of Measurements: 1:ia 2:ib 3:ic 4:theta_e 5:omega_m
    add_line(model_name, 'Measurements/1', 'FOC Controller/1', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/2', 'FOC Controller/2', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/3', 'FOC Controller/3', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/4', 'FOC Controller/4', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/5', 'FOC Controller/5', 'autorouting', 'smart');

    % Reference Generator --> FOC Controller
    add_line(model_name, 'Reference Generator/1', 'FOC Controller/6', 'autorouting', 'smart');
    add_line(model_name, 'Reference Generator/2', 'FOC Controller/7', 'autorouting', 'smart');

    % FOC Controller output G[6] --> Three-Phase Inverter gate signals
    add_line(model_name, 'FOC Controller/1', 'Three-Phase Inverter/g', ...
        'autorouting', 'smart');

    %% ── Output signal scopes and To Workspace blocks ─────────────────────────
    create_output_signals(model_name);

    % Connect output signals
    % omega_m (from Measurements port 5) --> Speed Scope
    add_line(model_name, 'Measurements/5', 'Speed Scope/1', 'autorouting', 'smart');
    % ia, ib, ic (ports 1-3) --> Phase Current Scope via Mux
    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_iabc'], ...
        'Inputs', '3', 'Position', [1390 80 1395 160]);
    add_line(model_name, 'Measurements/1', 'Mux_iabc/1', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/2', 'Mux_iabc/2', 'autorouting', 'smart');
    add_line(model_name, 'Measurements/3', 'Mux_iabc/3', 'autorouting', 'smart');
    add_line(model_name, 'Mux_iabc/1', 'Phase Current Scope/1', 'autorouting', 'smart');
    % id, iq (from FOC Controller ports 2, 3) --> dq Current Scope via Mux
    add_block('simulink/Signal Routing/Mux', [model_name '/Mux_idq'], ...
        'Inputs', '2', 'Position', [1390 200 1395 260]);
    add_line(model_name, 'FOC Controller/2', 'Mux_idq/1', 'autorouting', 'smart');
    add_line(model_name, 'FOC Controller/3', 'Mux_idq/2', 'autorouting', 'smart');
    add_line(model_name, 'Mux_idq/1', 'dq Current Scope/1', 'autorouting', 'smart');

    % To Workspace
    add_line(model_name, 'Measurements/5', 'omega_m/1', 'autorouting', 'smart');
    add_line(model_name, 'Mux_iabc/1',     'iabc/1',    'autorouting', 'smart');
    add_line(model_name, 'Mux_idq/1',      'idq/1',     'autorouting', 'smart');

    %% Annotation
    try
        ann = Simulink.Annotation([model_name '/PMSM FOC - Simscape Electrical']);
        ann.Position = [500 820];
    catch
    end

    %% Save model
    model_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'models');
    if ~exist(model_dir, 'dir')
        mkdir(model_dir);
    end
    save_system(model_name, fullfile(model_dir, [model_name '.slx']));
    fprintf('Model saved to: %s\n', fullfile(model_dir, [model_name '.slx']));
end

%% ===================== Helper Functions =====================

function create_foc_controller_subsystem(subsys, ctrl_params, inv_params)
%CREATE_FOC_CONTROLLER_SUBSYSTEM FOC controller using our own algorithm
%
%   Inputs:  1:ia  2:ib  3:ic  4:theta_e  5:omega_m  6:speed_ref  7:id_ref
%   Outputs: 1:G[6x1]  2:id_meas  3:iq_meas
%
%   Signal flow:
%     [ia,ib,ic] --> Clarke --> [i_alpha, i_beta]
%     [i_alpha, i_beta, theta_e] --> Park --> [id, iq]
%     speed_ref - omega_m --> Speed PI --> iq_ref
%     id_ref - id --> Id PI --> vd
%     iq_ref - iq --> Iq PI --> vq
%     [vd, vq, theta_e] --> InvPark --> [v_alpha, v_beta]
%     [v_alpha, v_beta] --> SVPWM --> [da, db, dc]  (duty 0..1)
%     [da, db, dc] --> PWM Generator --> G[6]  (gate signals for 6 switches)

    % === Input Ports ===
    add_block('built-in/Inport', [subsys '/ia'],        'Port', '1', 'Position', [30 63  60  77]);
    add_block('built-in/Inport', [subsys '/ib'],        'Port', '2', 'Position', [30 103  60 117]);
    add_block('built-in/Inport', [subsys '/ic'],        'Port', '3', 'Position', [30 143  60 157]);
    add_block('built-in/Inport', [subsys '/theta_e'],   'Port', '4', 'Position', [30 233  60 247]);
    add_block('built-in/Inport', [subsys '/omega_m'],   'Port', '5', 'Position', [30 323  60 337]);
    add_block('built-in/Inport', [subsys '/speed_ref'], 'Port', '6', 'Position', [30 413  60 427]);
    add_block('built-in/Inport', [subsys '/id_ref'],    'Port', '7', 'Position', [30 503  60 517]);

    % === Output Ports ===
    add_block('built-in/Outport', [subsys '/G'],        'Port', '1', 'Position', [1540 233 1570 247]);
    add_block('built-in/Outport', [subsys '/id_meas'],  'Port', '2', 'Position', [1540 323 1570 337]);
    add_block('built-in/Outport', [subsys '/iq_meas'],  'Port', '3', 'Position', [1540 413 1570 427]);

    % ── Clarke Transform: [ia, ib, ic] → [i_alpha, i_beta] ──────────────────
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_abc'], ...
        'Inputs', '3', 'Position', [120 60 125 160]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/Clarke'], ...
        'MATLABFcn', '[2/3*(u(1)-0.5*u(2)-0.5*u(3)); 2/3*(sqrt(3)/2*u(2)-sqrt(3)/2*u(3))]', ...
        'Position', [170 90 310 130]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_ab'], ...
        'Outputs', '2', 'Position', [340 90 345 130]);

    add_line(subsys, 'ia/1',     'Mux_abc/1');
    add_line(subsys, 'ib/1',     'Mux_abc/2');
    add_line(subsys, 'ic/1',     'Mux_abc/3');
    add_line(subsys, 'Mux_abc/1','Clarke/1');
    add_line(subsys, 'Clarke/1', 'Demux_ab/1');

    % ── Park Transform: [i_alpha, i_beta, theta_e] → [id, iq] ───────────────
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_park'], ...
        'Inputs', '3', 'Position', [390 80 395 180]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/Park'], ...
        'MATLABFcn', '[u(1)*cos(u(3))+u(2)*sin(u(3)); -u(1)*sin(u(3))+u(2)*cos(u(3))]', ...
        'Position', [430 100 570 140]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_dq'], ...
        'Outputs', '2', 'Position', [600 100 605 140]);

    add_line(subsys, 'Demux_ab/1',  'Mux_park/1');
    add_line(subsys, 'Demux_ab/2',  'Mux_park/2');
    add_line(subsys, 'theta_e/1',   'Mux_park/3');
    add_line(subsys, 'Mux_park/1',  'Park/1');
    add_line(subsys, 'Park/1',      'Demux_dq/1');

    % id, iq measurement outputs
    add_line(subsys, 'Demux_dq/1', 'id_meas/1');
    add_line(subsys, 'Demux_dq/2', 'iq_meas/1');

    % ── Speed PI: speed_ref [RPM] − omega_m [rad/s] → iq_ref ────────────────
    add_block('simulink/Math Operations/Gain', [subsys '/RPM2RadS'], ...
        'Gain', '2*pi/60', 'Position', [120 410 160 430]);
    add_block('simulink/Math Operations/Sum', [subsys '/Speed Error'], ...
        'Inputs', '+-', 'Position', [200 410 230 440]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Speed PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_speed), ...
        'I', num2str(ctrl_params.Ki_speed), ...
        'Position', [270 408 350 448]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Iq Sat'], ...
        'UpperLimit', num2str(ctrl_params.iq_max), ...
        'LowerLimit', num2str(-ctrl_params.iq_max), ...
        'Position', [390 410 440 440]);

    add_line(subsys, 'speed_ref/1',  'RPM2RadS/1');
    add_line(subsys, 'RPM2RadS/1',   'Speed Error/1');
    add_line(subsys, 'omega_m/1',    'Speed Error/2');
    add_line(subsys, 'Speed Error/1','Speed PI/1');
    add_line(subsys, 'Speed PI/1',   'Iq Sat/1');

    % ── d-axis Current PI: id_ref − id → vd ─────────────────────────────────
    add_block('simulink/Math Operations/Sum', [subsys '/Id Error'], ...
        'Inputs', '+-', 'Position', [660 170 690 200]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Id PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_id), ...
        'I', num2str(ctrl_params.Ki_id), ...
        'Position', [730 168 810 208]);

    add_line(subsys, 'id_ref/1',    'Id Error/1');
    add_line(subsys, 'Demux_dq/1',  'Id Error/2');
    add_line(subsys, 'Id Error/1',  'Id PI/1');

    % ── q-axis Current PI: iq_ref − iq → vq ─────────────────────────────────
    add_block('simulink/Math Operations/Sum', [subsys '/Iq Error'], ...
        'Inputs', '+-', 'Position', [660 260 690 290]);
    add_block('simulink/Continuous/PID Controller', [subsys '/Iq PI'], ...
        'Controller', 'PI', ...
        'P', num2str(ctrl_params.Kp_iq), ...
        'I', num2str(ctrl_params.Ki_iq), ...
        'Position', [730 258 810 298]);

    add_line(subsys, 'Iq Sat/1',    'Iq Error/1');
    add_line(subsys, 'Demux_dq/2',  'Iq Error/2');
    add_line(subsys, 'Iq Error/1',  'Iq PI/1');

    % ── Inverse Park: [vd, vq, theta_e] → [v_alpha, v_beta] ─────────────────
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_invpark'], ...
        'Inputs', '3', 'Position', [860 190 865 290]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/InvPark'], ...
        'MATLABFcn', '[u(1)*cos(u(3))-u(2)*sin(u(3)); u(1)*sin(u(3))+u(2)*cos(u(3))]', ...
        'Position', [900 220 1040 260]);
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_vab'], ...
        'Outputs', '2', 'Position', [1070 220 1075 260]);

    add_line(subsys, 'Id PI/1',       'Mux_invpark/1');
    add_line(subsys, 'Iq PI/1',       'Mux_invpark/2');
    add_line(subsys, 'theta_e/1',     'Mux_invpark/3');
    add_line(subsys, 'Mux_invpark/1', 'InvPark/1');
    add_line(subsys, 'InvPark/1',     'Demux_vab/1');

    % ── SVPWM: [v_alpha, v_beta, Vdc] → [da, db, dc] ────────────────────────
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_svpwm'], ...
        'Inputs', '3', 'Position', [1110 215 1115 285]);
    add_block('built-in/Constant', [subsys '/Vdc_const'], ...
        'Value', num2str(inv_params.Vdc), ...
        'Position', [1060 280 1100 300]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/SVPWM'], ...
        'MATLABFcn', 'svpwm_inline(u(1), u(2), u(3))', ...
        'Position', [1150 225 1280 265]);

    add_line(subsys, 'Demux_vab/1', 'Mux_svpwm/1');
    add_line(subsys, 'Demux_vab/2', 'Mux_svpwm/2');
    add_line(subsys, 'Vdc_const/1', 'Mux_svpwm/3');
    add_line(subsys, 'Mux_svpwm/1', 'SVPWM/1');

    % ── PWM Generator: [da, db, dc] → G[6] ───────────────────────────────────
    % Compare each duty cycle with a sawtooth carrier at fsw to produce
    % gate signals for the six inverter switches:
    %   g1 = (da >= carrier),  g2 = NOT g1   (phase A upper / lower)
    %   g3 = (db >= carrier),  g4 = NOT g3   (phase B upper / lower)
    %   g5 = (dc >= carrier),  g6 = NOT g5   (phase C upper / lower)
    %
    % NOTE: g2 = 1-g1 implements an idealized complementary switching with
    % zero dead time.  A real drive must insert a short dead-time interval
    % (both switches OFF) to prevent a DC-bus shoot-through.  Dead-time
    % insertion can be added here by replacing "g2=1-g1" with a Discrete
    % Pulse Generator or a Transport Delay before the inversion.
    %
    % The PWM Generator MATLAB Fcn takes [da, db, dc, carrier] as input.
    add_block('simulink/Signal Routing/Demux', [subsys '/Demux_duty'], ...
        'Outputs', '3', 'Position', [1320 225 1325 265]);
    add_line(subsys, 'SVPWM/1', 'Demux_duty/1');

    % Sawtooth carrier at switching frequency (normalized 0..1)
    add_block('simulink/Sources/Signal Generator', [subsys '/Carrier'], ...
        'WaveForm',  'sawtooth', ...
        'Frequency', num2str(inv_params.fsw), ...
        'Amplitude',  '0.5', ...
        'Bias',       '0.5', ...
        'Position',  [1320 290 1390 330]);

    % PWM comparison via MATLAB Fcn (inline, no external dependency)
    % Input: u(1..3) = duty cycles; u(4) = carrier
    % Output: [g1; g2; g3; g4; g5; g6] as double (0 or 1)
    pwm_fcn = [ ...
        'g1=double(u(1)>=u(4)); g2=1-g1; ' ...
        'g3=double(u(2)>=u(4)); g4=1-g3; ' ...
        'g5=double(u(3)>=u(4)); g6=1-g5; ' ...
        '[g1;g2;g3;g4;g5;g6]'];
    add_block('simulink/Signal Routing/Mux', [subsys '/Mux_pwm'], ...
        'Inputs', '4', 'Position', [1410 215 1415 335]);
    add_block('simulink/User-Defined Functions/MATLAB Fcn', [subsys '/PWM Gen'], ...
        'MATLABFcn', pwm_fcn, ...
        'Position', [1450 230 1530 250]);

    add_line(subsys, 'Demux_duty/1', 'Mux_pwm/1');
    add_line(subsys, 'Demux_duty/2', 'Mux_pwm/2');
    add_line(subsys, 'Demux_duty/3', 'Mux_pwm/3');
    add_line(subsys, 'Carrier/1',    'Mux_pwm/4');
    add_line(subsys, 'Mux_pwm/1',   'PWM Gen/1');
    add_line(subsys, 'PWM Gen/1',   'G/1');
end

function create_measurement_subsystem(subsys, motor_params)
%CREATE_MEASUREMENT_SUBSYSTEM Our own encoder module
%
%   Converts raw sensor signals from Simscape blocks into FOC-ready signals.
%
%   Inputs:  1:ia  2:ib  3:ic  4:theta_m [rad]  5:omega_m [rad/s]
%   Outputs: 1:ia_meas  2:ib_meas  3:ic_meas  4:theta_e [rad]  5:omega_m [rad/s]
%
%   theta_e = p * theta_m   (electrical angle from mechanical angle)
%   omega_m pass-through    (no derivative block — avoids numerical noise)

    add_block('built-in/Inport',  [subsys '/ia'],       'Port', '1', 'Position', [30 63  60  77]);
    add_block('built-in/Inport',  [subsys '/ib'],       'Port', '2', 'Position', [30 103  60 117]);
    add_block('built-in/Inport',  [subsys '/ic'],       'Port', '3', 'Position', [30 143  60 157]);
    add_block('built-in/Inport',  [subsys '/theta_m'],  'Port', '4', 'Position', [30 233  60 247]);
    add_block('built-in/Inport',  [subsys '/omega_m_in'], 'Port', '5', 'Position', [30 323  60 337]);

    add_block('built-in/Outport', [subsys '/ia_meas'],  'Port', '1', 'Position', [240 63  270  77]);
    add_block('built-in/Outport', [subsys '/ib_meas'],  'Port', '2', 'Position', [240 103  270 117]);
    add_block('built-in/Outport', [subsys '/ic_meas'],  'Port', '3', 'Position', [240 143  270 157]);
    add_block('built-in/Outport', [subsys '/theta_e'],  'Port', '4', 'Position', [240 233  270 247]);
    add_block('built-in/Outport', [subsys '/omega_m'],  'Port', '5', 'Position', [240 323  270 337]);

    % Phase current pass-through
    add_line(subsys, 'ia/1', 'ia_meas/1');
    add_line(subsys, 'ib/1', 'ib_meas/1');
    add_line(subsys, 'ic/1', 'ic_meas/1');

    % Electrical angle: theta_e = pole_pairs * theta_m
    add_block('simulink/Math Operations/Gain', [subsys '/PoleP'], ...
        'Gain', num2str(motor_params.p), ...
        'Position', [120 228 170 252]);
    add_line(subsys, 'theta_m/1', 'PoleP/1');
    add_line(subsys, 'PoleP/1',   'theta_e/1');

    % Speed pass-through
    add_line(subsys, 'omega_m_in/1', 'omega_m/1');
end

function create_reference_generator(subsys, ref_params)
%CREATE_REFERENCE_GENERATOR Speed ramp reference + id_ref = 0 (MTPA)
%   Output 1: speed_ref [RPM, ramped]   Output 2: id_ref [A]

    add_block('built-in/Outport', [subsys '/speed_ref'], 'Port', '1', 'Position', [240 33 270 47]);
    add_block('built-in/Outport', [subsys '/id_ref'],    'Port', '2', 'Position', [240 93 270 107]);

    add_block('simulink/Sources/Ramp', [subsys '/Speed Ramp'], ...
        'Slope', num2str(ref_params.speed_ref / ref_params.speed_ramp_time), ...
        'Position', [50 28 120 52]);
    add_block('simulink/Discontinuities/Saturation', [subsys '/Speed Sat'], ...
        'UpperLimit', num2str(ref_params.speed_ref), ...
        'LowerLimit', '0', ...
        'Position', [160 28 220 52]);
    add_block('built-in/Constant', [subsys '/Id Ref'], ...
        'Value', num2str(ref_params.id_ref), ...
        'Position', [100 88 150 112]);

    add_line(subsys, 'Speed Ramp/1', 'Speed Sat/1');
    add_line(subsys, 'Speed Sat/1',  'speed_ref/1');
    add_line(subsys, 'Id Ref/1',     'id_ref/1');
end

function create_output_signals(model_name)
%CREATE_OUTPUT_SIGNALS Add scopes and To Workspace blocks for signal logging

    % ── Scopes ────────────────────────────────────────────────────────────────
    add_block('simulink/Sinks/Scope', [model_name '/Speed Scope'], ...
        'NumInputPorts', '1', ...
        'Position', [1450 50 1510 100]);
    set_param([model_name '/Speed Scope'], 'Title', 'Mechanical Speed [rad/s]');

    add_block('simulink/Sinks/Scope', [model_name '/Phase Current Scope'], ...
        'NumInputPorts', '1', ...
        'Position', [1450 140 1510 190]);
    set_param([model_name '/Phase Current Scope'], 'Title', 'Phase Currents ia/ib/ic [A]');

    add_block('simulink/Sinks/Scope', [model_name '/dq Current Scope'], ...
        'NumInputPorts', '1', ...
        'Position', [1450 230 1510 280]);
    set_param([model_name '/dq Current Scope'], 'Title', 'dq Currents id/iq [A]');

    % ── To Workspace ──────────────────────────────────────────────────────────
    add_block('simulink/Sinks/To Workspace', [model_name '/omega_m'], ...
        'VariableName', 'out_omega_m', ...
        'SaveFormat',   'Array', ...
        'Position', [1450 320 1530 360]);
    add_block('simulink/Sinks/To Workspace', [model_name '/iabc'], ...
        'VariableName', 'out_iabc', ...
        'SaveFormat',   'Array', ...
        'Position', [1450 380 1530 420]);
    add_block('simulink/Sinks/To Workspace', [model_name '/idq'], ...
        'VariableName', 'out_idq', ...
        'SaveFormat',   'Array', ...
        'Position', [1450 440 1530 480]);
end
