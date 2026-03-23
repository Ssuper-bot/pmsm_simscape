%% PMSM FOC Simscape Simulation - Main Setup Script
%
% Creates a Simulink/Simscape Electrical model for PMSM Field-Oriented Control.
%
% Model architecture
% ------------------
% Physical domain (Simscape Electrical):
%   DC Voltage Source --> Three-Phase Inverter (ee_lib)
%   Three-Phase Inverter (a/b/c) --> Phase Current Sensors --> PMSM (ee_lib)
%   PMSM rotor --> Ideal Rotational Motion Sensor + Load Torque Source --> Mech Ref
%
% Signal domain (Simulink):
%   Phase Current Sensors (ia, ib, ic)
%   Rotational Motion Sensor (theta_m, omega_m)
%   Measurements subsystem  -- our own encoder module --
%     theta_e = p * theta_m, omega_m pass-through
%   FOC Controller          -- our own algorithm --
%     Clarke -> Park -> Speed PI -> Current PI (d/q) -> InvPark -> SVPWM -> PWM Gen
%     Output G[6]: gate signals for the 6 inverter switches
%   Reference Generator: speed ramp (RPM) + id_ref = 0 (MTPA)
%
% Output signals logged:
%   Scopes: Speed [rad/s], Phase Currents ia/ib/ic [A], dq Currents id/iq [A]
%   To Workspace: out_omega_m, out_iabc, out_idq
%
% Requires: Simulink, Simscape, Simscape Electrical (ee_lib)
%
% Usage (in MATLAB command window):
%   >> cd('/path/to/pmsm_simscape')
%   >> setup_project       % Add paths (run once per session)
%   >> pmsm_foc_simscape   % Run this script to build and open the model

%% Check Simulink and Simscape Electrical availability
if ~exist('bdIsLoaded', 'file') && ~exist('new_system', 'file')
    error(['pmsm_foc_simscape requires Simulink.\n' ...
           'If you don''t have Simulink, use run_pmsm_simulation instead.\n' ...
           'Usage:\n  >> run_pmsm_simulation']);
end

% Warn if Simscape Electrical is not available (but continue to allow path setup).
% ver('simscape_electrical') is the reliable check; ee_lib is a library name, not a directory.
if isempty(ver('simscape_electrical'))
    warning(['Simscape Electrical does not appear to be installed.\n' ...
             'The model build will fail when attempting to add PMSM / Inverter blocks.\n' ...
             'Ensure you have a valid Simscape Electrical license.']);
end

%% Add paths
script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..');
addpath(matlab_dir);                          % +pmsm_lib package
addpath(fullfile(matlab_dir, 'scripts'));      % build_sfun_foc etc.
addpath(script_dir);                          % create_pmsm_foc_model, svpwm_inline
rehash path;

%% Clear workspace parameters
clear motor_params inv_params ctrl_params sim_params ref_params;

%% ── Motor Parameters (typical surface-mounted PMSM) ─────────────────────────
motor_params = struct();
motor_params.Rs       = 0.5;        % Stator resistance [Ohm]
motor_params.Ld       = 1.4e-3;     % d-axis inductance [H]
motor_params.Lq       = 1.4e-3;     % q-axis inductance [H]  (Ld=Lq for SPMSM)
motor_params.flux_pm  = 0.0577;     % Permanent magnet flux linkage [Wb]
motor_params.p        = 4;          % Number of pole pairs
motor_params.J        = 1.74e-5;    % Rotor inertia [kg*m^2]
motor_params.B        = 1e-4;       % Viscous damping [N*m*s/rad]

%% ── Inverter Parameters ──────────────────────────────────────────────────────
inv_params = struct();
inv_params.Vdc       = 48;          % DC bus voltage [V]
inv_params.fsw       = 20e3;        % Switching frequency [Hz]
inv_params.Tsw       = 1/inv_params.fsw;   % Switching period [s]
inv_params.dead_time = 1e-6;        % Dead time [s] (for reference; not yet modelled)

%% ── Controller Parameters (PI gains tuned for the motor above) ───────────────
ctrl_params = struct();
% d-axis current loop PI
ctrl_params.Kp_id    = 5.0;
ctrl_params.Ki_id    = 1000.0;
% q-axis current loop PI
ctrl_params.Kp_iq    = 5.0;
ctrl_params.Ki_iq    = 1000.0;
% Speed loop PI
ctrl_params.Kp_speed = 0.5;
ctrl_params.Ki_speed = 10.0;
% Current limits
ctrl_params.id_max   = 10.0;       % Max d-axis current [A]
ctrl_params.iq_max   = 10.0;       % Max q-axis current [A]
ctrl_params.speed_max = 3000;      % Max speed [RPM]

%% ── Simulation Parameters ────────────────────────────────────────────────────
sim_params = struct();
% ode23t is recommended for Simscape stiff DAE systems with switching inverters.
% ode15s is an alternative for highly stiff systems.
sim_params.solver    = 'ode23t';
sim_params.t_end     = 0.5;         % Simulation duration [s]
% Max step: 0.5 × switching period gives ~2 samples per PWM period, which is
% sufficient to resolve the switched inverter waveforms with ode23t.
% Tighten to 0.1–0.2 × Tsw if higher PWM fidelity is needed.
sim_params.max_step  = 0.5 * inv_params.Tsw;   % 2.5e-5 s @ 20 kHz
sim_params.Ts_control = inv_params.Tsw;         % Control sample time [s]
sim_params.Ts_speed   = 10 * inv_params.Tsw;    % Speed loop sample time [s]

%% ── Reference Signals ────────────────────────────────────────────────────────
ref_params = struct();
ref_params.speed_ref       = 1000;  % Target speed [RPM]
ref_params.speed_ramp_time = 0.1;   % Time to ramp from 0 to speed_ref [s]
ref_params.load_torque     = 0.1;   % Step load torque magnitude [N*m]
ref_params.load_step_time  = 0.3;   % Time at which load step is applied [s]
ref_params.id_ref          = 0;     % d-axis current reference [A] (MTPA: id=0)

%% Export parameters to base workspace (available inside Simulink model)
assignin('base', 'motor_params', motor_params);
assignin('base', 'inv_params',   inv_params);
assignin('base', 'ctrl_params',  ctrl_params);
assignin('base', 'sim_params',   sim_params);
assignin('base', 'ref_params',   ref_params);

%% ── Build Simscape Electrical model ──────────────────────────────────────────
model_name = 'pmsm_foc_simscape_ee';
model_path = fullfile(fileparts(mfilename('fullpath')), '..', 'models', [model_name '.slx']);

if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

if exist(model_path, 'file')
    fprintf('Deleting stale model to regenerate: %s\n', model_path);
    delete(model_path);
end

fprintf('Building Simscape Electrical FOC model...\n');
create_pmsm_foc_model(model_name, motor_params, inv_params, ctrl_params, sim_params, ref_params);

%% ── Summary ──────────────────────────────────────────────────────────────────
fprintf('\n=== Setup Complete ===\n');
fprintf('Model : %s\n', model_name);
fprintf('Motor : Rs=%.3f Ohm, Ld=Lq=%.2e H, lambda_pm=%.4f Wb, p=%d\n', ...
    motor_params.Rs, motor_params.Ld, motor_params.flux_pm, motor_params.p);
fprintf('Inverter : Vdc=%d V, fsw=%d kHz\n', inv_params.Vdc, inv_params.fsw/1e3);
fprintf('Reference: %d RPM target, %.2f N*m load step at t=%.2f s\n', ...
    ref_params.speed_ref, ref_params.load_torque, ref_params.load_step_time);
fprintf('Solver   : %s, max_step=%.2e s, t_end=%.2f s\n', ...
    sim_params.solver, sim_params.max_step, sim_params.t_end);
fprintf('\nTo run: sim(''%s'')\n', model_name);
fprintf('Output variables: out_omega_m, out_iabc, out_idq (in base workspace)\n');
