%% PMSM FOC Standalone Simulation (No Simscape Required)
% This script simulates the PMSM FOC control loop using ODE integration.
% Useful for validating the algorithm before Simscape integration.
%
% Usage (in MATLAB command window):
%   >> cd('path/to/pmsm_simscape/matlab/scripts')
%   >> run_pmsm_simulation
%
% Uses the same C++ MEX controller core as the Simulink S-Function path.

clear; clc; close all;

%% Add +pmsm_lib package to path
script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..');
addpath(matlab_dir);
addpath(script_dir);
rehash path;

%% ===== Motor Parameters (typical PMSM) =====
motor_params = struct();
motor_params.Rs = 0.5;           % Stator resistance [Ohm]
motor_params.Ld = 1.4e-3;        % d-axis inductance [H]
motor_params.Lq = 1.4e-3;        % q-axis inductance [H]
motor_params.flux_pm = 0.0577;   % Permanent magnet flux linkage [Wb]
motor_params.p = 4;              % Number of pole pairs
motor_params.J = 1.74e-5;        % Rotor inertia [kg*m^2]
motor_params.B = 1e-4;           % Viscous damping [N*m*s]

%% ===== Inverter Parameters =====
inv_params = struct();
inv_params.Vdc = 24;             % DC bus voltage [V]
inv_params.fsw = 20e3;           % Switching frequency [Hz]

%% ===== Controller Parameters =====
ctrl_params = struct();
ctrl_params.omega_ci = 2*pi*(inv_params.fsw / 10.0);  % Current-loop BW [rad/s]
ctrl_params.omega_cs = ctrl_params.omega_ci / 10.0;   % Speed-loop BW [rad/s]
ctrl_params.Kp_id = 0.0;
ctrl_params.Ki_id = 0.0;
ctrl_params.Kp_iq = 0.0;
ctrl_params.Ki_iq = 0.0;
ctrl_params.Kp_speed = 0.0;
ctrl_params.Ki_speed = 0.0;
ctrl_params.iq_max = 10.0;       % Max q-axis current [A]
ctrl_params.id_max = 10.0;       % Max d-axis current [A]
ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params);
ctrl_params.Vdc = inv_params.Vdc;

%% ===== Simulation Parameters =====
sim_params = struct();
sim_params.Ts_control = 1/inv_params.fsw;  % Control sample time [s]
sim_params.t_end = 0.5;                     % Simulation end time [s]

%% ===== Reference Signals =====
ref_params = struct();
ref_params.speed_ref = 1000;      % Reference speed [RPM]
ref_params.speed_ramp_time = 0.1; % Speed ramp time [s]
ref_params.load_torque = 0.1;     % Load torque [N*m]
ref_params.load_step_time = 0.3;  % Load step time [s]
ref_params.id_ref = 0;            % d-axis current reference (MTPA: id=0)

fprintf('=== PMSM FOC Standalone Simulation ===\n');
fprintf('Motor: Rs=%.3f Ohm, Ld=%.3e H, flux=%.4f Wb, %d pole pairs\n', ...
    motor_params.Rs, motor_params.Ld, motor_params.flux_pm, motor_params.p);
fprintf('Target speed: %d RPM, Vdc: %d V\n', ref_params.speed_ref, inv_params.Vdc);
fprintf('Auto PI: omega_ci=%.1f rad/s, omega_cs=%.1f rad/s\n', ...
    ctrl_params.omega_ci, ctrl_params.omega_cs);

%% Build controller MEX
ctrl_params.Ts = sim_params.Ts_control;
mex_path = fullfile(script_dir, ['foc_controller_mex.' mexext]);
if ~exist(mex_path, 'file')
    fprintf('Building C++ controller MEX for standalone simulation...\n');
    build_foc_mex(script_dir);
end

clear('foc_controller_mex');

%% Initial conditions [id, iq, omega_m, theta_m]
% theta_m = mechanical angle; theta_e = p * theta_m
x0 = [0; 0; 0; 0];

%% Time vector
dt = sim_params.Ts_control;
t_end = sim_params.t_end;
t = 0:dt:t_end;
N = length(t);

%% Preallocate output arrays
x_log = zeros(4, N);
u_log = zeros(2, N);   % [vd, vq]
ref_log = zeros(2, N); % [speed_ref, iq_ref]
duty_log = zeros(3, N); % [da, db, dc]

%% Simulation loop
x = x0;
for k = 1:N
    tk = t(k);
    
    % Current state
    id = x(1);
    iq = x(2);
    omega_m = x(3);
    theta_m = x(4);
    theta_e = motor_params.p * theta_m;  % Electrical angle
    
    %% Generate references
    % Speed reference with ramp
    speed_ref_rpm = min(ref_params.speed_ref, ...
        ref_params.speed_ref * tk / ref_params.speed_ramp_time);
    speed_ref_rad = speed_ref_rpm * 2 * pi / 60;
    
    % Load torque step
    if tk >= ref_params.load_step_time
        Te_load = ref_params.load_torque;
    else
        Te_load = 0;
    end

    torque_ref = 0.0;
    
    %% FOC Controller
    [i_alpha, i_beta] = pmsm_lib.inv_park_transform(id, iq, theta_e);
    ia = i_alpha;
    ib = -0.5 * i_alpha + (sqrt(3) / 2) * i_beta;
    ic = -0.5 * i_alpha - (sqrt(3) / 2) * i_beta;

    ctrl_output = foc_controller_mex( ...
        ia, ib, ic, theta_e, omega_m, speed_ref_rad, ...
        ref_params.id_ref, dt, ctrl_params, motor_params, torque_ref);

    vd = ctrl_output(6);
    vq = ctrl_output(7);
    iq_ref = ctrl_output(8);
    
    %% SVPWM (for logging)
    [v_alpha, v_beta] = pmsm_lib.inv_park_transform(vd, vq, theta_e);
    [da, db, dc] = pmsm_lib.svpwm_modulator(v_alpha, v_beta, inv_params.Vdc);
    
    %% Log
    x_log(:, k) = x;
    u_log(:, k) = [vd; vq];
    ref_log(:, k) = [speed_ref_rad; iq_ref];
    duty_log(:, k) = [da; db; dc];
    
    %% Plant Model (Forward Euler integration)
    [did, diq, domega, dtheta] = pmsm_lib.pmsm_dq_model( ...
        vd, vq, id, iq, omega_m, Te_load, motor_params);
    
    x = x + dt * [did; diq; domega; dtheta];
    
    % Wrap theta_m to [0, 2*pi]
    x(4) = mod(x(4), 2*pi);
end

%% ===== Plot Results =====
figure('Name', 'PMSM FOC Simulation', 'Position', [100 100 1200 800]);

% Speed
subplot(3,2,1);
plot(t, x_log(3,:)*60/(2*pi), 'b', t, ref_log(1,:)*60/(2*pi), 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Speed [RPM]');
title('Rotor Speed'); legend('Actual', 'Reference'); grid on;

% d-axis current
subplot(3,2,2);
plot(t, x_log(1,:), 'b', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('i_d [A]');
title('d-axis Current'); grid on;

% q-axis current
subplot(3,2,3);
plot(t, x_log(2,:), 'b', t, ref_log(2,:), 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('i_q [A]');
title('q-axis Current'); legend('Actual', 'Reference'); grid on;

% Voltages
subplot(3,2,4);
plot(t, u_log(1,:), 'b', t, u_log(2,:), 'r', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Voltage [V]');
title('dq-axis Voltages'); legend('v_d', 'v_q'); grid on;

% Duty cycles
subplot(3,2,5);
plot(t, duty_log(1,:), 'r', t, duty_log(2,:), 'g', t, duty_log(3,:), 'b', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('Duty Cycle');
title('SVPWM Duty Cycles'); legend('a', 'b', 'c'); grid on;

% Electrical angle
subplot(3,2,6);
plot(t, mod(motor_params.p * x_log(4,:), 2*pi), 'b', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('\theta_e [rad]');
title('Electrical Angle'); grid on;

sgtitle('PMSM FOC Simulation Results');

fprintf('Simulation complete.\n');
fprintf('Final speed: %.1f RPM (ref: %.1f RPM)\n', ...
    x_log(3,end)*60/(2*pi), ref_log(1,end)*60/(2*pi));

function ctrl_params = derive_pi_ctrl_params(ctrl_params, motor_params)
Rs = max(motor_params.Rs, 1e-12);
Ld = max(motor_params.Ld, 1e-12);
Lq = max(motor_params.Lq, 1e-12);
J = max(motor_params.J, 1e-12);
B = max(motor_params.B, 1e-12);
Kt = max(1.5 * motor_params.p * motor_params.flux_pm, 1e-12);

ctrl_params.Kp_id = Ld * ctrl_params.omega_ci;
ctrl_params.Ki_id = Rs * ctrl_params.omega_ci;
ctrl_params.Kp_iq = Lq * ctrl_params.omega_ci;
ctrl_params.Ki_iq = Rs * ctrl_params.omega_ci;

ctrl_params.Kp_speed = (J * ctrl_params.omega_cs) / Kt;
ctrl_params.Ki_speed = (B * ctrl_params.omega_cs) / Kt;
end
