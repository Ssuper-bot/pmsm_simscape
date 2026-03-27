function results = check_foc_angle_contract(varargin)
%CHECK_FOC_ANGLE_CONTRACT Validate abc/current and theta_e sign conventions.
%
% This is a minimal, model-free sanity check for the FOC contract used by
% the project. It does not touch Simscape, PWM, inverter, or motor blocks.
%
% Usage:
%   results = check_foc_angle_contract
%   results = check_foc_angle_contract('CurrentAmplitude', 5, 'Plot', true)

parser = inputParser;
parser.addParameter('CurrentAmplitude', 5, @(x) validateattributes(x, {'numeric'}, {'scalar', 'real', 'positive'}));
parser.addParameter('Plot', true, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

setup_project;

n = 400;
theta = linspace(0, 2*pi, n);
current_amp = opts.CurrentAmplitude;

% Case 1: flux-aligned current vector, expected id > 0 and iq ~= 0.
alpha_flux = current_amp * cos(theta);
beta_flux = current_amp * sin(theta);
[ia_flux, ib_flux, ic_flux] = alpha_beta_to_abc(alpha_flux, beta_flux);
[id_flux, iq_flux] = apply_park(ia_flux, ib_flux, ic_flux, theta);

% Case 2: torque-producing current vector, expected id ~= 0 and iq > 0.
alpha_torque = -current_amp * sin(theta);
beta_torque =  current_amp * cos(theta);
[ia_torque, ib_torque, ic_torque] = alpha_beta_to_abc(alpha_torque, beta_torque);
[id_torque, iq_torque] = apply_park(ia_torque, ib_torque, ic_torque, theta);

results = struct();
results.flux_case.mean_id = mean(id_flux);
results.flux_case.mean_iq = mean(iq_flux);
results.flux_case.peak_id_error = max(abs(id_flux - current_amp));
results.flux_case.peak_iq_error = max(abs(iq_flux));
results.flux_case.abc = [ia_flux(:), ib_flux(:), ic_flux(:)];

results.torque_case.mean_id = mean(id_torque);
results.torque_case.mean_iq = mean(iq_torque);
results.torque_case.peak_id_error = max(abs(id_torque));
results.torque_case.peak_iq_error = max(abs(iq_torque - current_amp));
results.torque_case.abc = [ia_torque(:), ib_torque(:), ic_torque(:)];

fprintf('\n=== FOC Angle Contract Check ===\n');
fprintf('Convention in this repo:\n');
fprintf('  id =  alpha*cos(theta_e) + beta*sin(theta_e)\n');
fprintf('  iq = -alpha*sin(theta_e) + beta*cos(theta_e)\n\n');
fprintf('Flux-aligned case:   mean(id)=%.4f A, mean(iq)=%.4f A\n', ...
    results.flux_case.mean_id, results.flux_case.mean_iq);
fprintf('  peak |id - I| = %.4e, peak |iq| = %.4e\n', ...
    results.flux_case.peak_id_error, results.flux_case.peak_iq_error);
fprintf('Torque-aligned case: mean(id)=%.4f A, mean(iq)=%.4f A\n', ...
    results.torque_case.mean_id, results.torque_case.mean_iq);
fprintf('  peak |id| = %.4e, peak |iq - I| = %.4e\n', ...
    results.torque_case.peak_id_error, results.torque_case.peak_iq_error);

if exist('foc_controller_mex', 'file') == 3
    sample_idx = round(n / 7);
    mex_out = foc_controller_mex( ...
        ia_torque(sample_idx), ib_torque(sample_idx), ic_torque(sample_idx), ...
        theta(sample_idx), 0, 0, 0, 1/20000);
    results.torque_case.mex_id = mex_out(4);
    results.torque_case.mex_iq = mex_out(5);
    fprintf('\nFOC MEX spot check on torque-aligned sample:\n');
    fprintf('  mex id = %.4f A, mex iq = %.4f A\n', mex_out(4), mex_out(5));
else
    fprintf('\nFOC MEX spot check skipped: foc_controller_mex not built.\n');
end

if opts.Plot
    figure('Name', 'FOC Angle Contract Check', 'Position', [100 100 1200 720]);

    subplot(2, 2, 1);
    plot(theta, ia_flux, theta, ib_flux, theta, ic_flux, 'LineWidth', 1.0);
    grid on;
    title('Flux-Aligned abc');
    xlabel('theta_e [rad]');
    ylabel('Current [A]');
    legend('i_a', 'i_b', 'i_c', 'Location', 'best');

    subplot(2, 2, 2);
    plot(theta, id_flux, theta, iq_flux, 'LineWidth', 1.2);
    grid on;
    title('Flux-Aligned dq');
    xlabel('theta_e [rad]');
    ylabel('Current [A]');
    legend('i_d', 'i_q', 'Location', 'best');

    subplot(2, 2, 3);
    plot(theta, ia_torque, theta, ib_torque, theta, ic_torque, 'LineWidth', 1.0);
    grid on;
    title('Torque-Aligned abc');
    xlabel('theta_e [rad]');
    ylabel('Current [A]');
    legend('i_a', 'i_b', 'i_c', 'Location', 'best');

    subplot(2, 2, 4);
    plot(theta, id_torque, theta, iq_torque, 'LineWidth', 1.2);
    grid on;
    title('Torque-Aligned dq');
    xlabel('theta_e [rad]');
    ylabel('Current [A]');
    legend('i_d', 'i_q', 'Location', 'best');
end

end

function [id, iq] = apply_park(ia, ib, ic, theta)
[alpha, beta] = arrayfun(@pmsm_lib.clarke_transform, ia, ib, ic);
[id, iq] = arrayfun(@pmsm_lib.park_transform, alpha, beta, theta);
end

function [ia, ib, ic] = alpha_beta_to_abc(alpha, beta)
ia = alpha;
ib = -0.5 .* alpha + (sqrt(3) / 2) .* beta;
ic = -0.5 .* alpha - (sqrt(3) / 2) .* beta;
end