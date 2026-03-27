function summary = diagnose_foc_interface_cases(varargin)
%DIAGNOSE_FOC_INTERFACE_CASES Compare common FOC interface mismatch cases.
%
% This script does not run the plant. It compares a few fixed hypotheses for
% theta and phase-current wiring against the repo's current Clarke/Park
% convention. The goal is to identify which mismatch pattern best explains a
% measured symptom such as negative id or dq cross-coupling.

parser = inputParser;
parser.addParameter('CurrentAmplitude', 5, @(x) validateattributes(x, {'numeric'}, {'scalar', 'real', 'positive'}));
parser.addParameter('Plot', false, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

setup_project;

n = 400;
theta = linspace(0, 2*pi, n);
current_amp = opts.CurrentAmplitude;

alpha_nom = -current_amp * sin(theta);
beta_nom = current_amp * cos(theta);
[ia_nom, ib_nom, ic_nom] = alpha_beta_to_abc(alpha_nom, beta_nom);

cases = {
    'nominal',              ia_nom,  ib_nom,  ic_nom,  theta;
    'theta_neg',            ia_nom,  ib_nom,  ic_nom, -theta;
    'theta_plus_pi_over_2', ia_nom,  ib_nom,  ic_nom,  theta + pi/2;
    'theta_minus_pi_over_2',ia_nom,  ib_nom,  ic_nom,  theta - pi/2;
    'current_neg',         -ia_nom, -ib_nom, -ic_nom,  theta;
    'swap_a_b',             ib_nom,  ia_nom,  ic_nom,  theta;
    'swap_b_c',             ia_nom,  ic_nom,  ib_nom,  theta;
    'swap_c_a',             ic_nom,  ib_nom,  ia_nom,  theta;
};

rows = cell(size(cases, 1), 6);
waveforms = struct();

for idx = 1:size(cases, 1)
    name = cases{idx, 1};
    ia = cases{idx, 2};
    ib = cases{idx, 3};
    ic = cases{idx, 4};
    theta_case = cases{idx, 5};

    [id, iq] = apply_park(ia, ib, ic, theta_case);

    rows(idx, :) = {
        name, ...
        mean(id), ...
        mean(iq), ...
        mean(abs(id)), ...
        mean(abs(iq)), ...
        max(abs(ia + ib + ic)) ...
    };

    waveforms.(matlab.lang.makeValidName(name)) = struct( ...
        'theta', theta_case, 'ia', ia, 'ib', ib, 'ic', ic, 'id', id, 'iq', iq);
end

summary = cell2table(rows, 'VariableNames', ...
    {'case_name', 'mean_id', 'mean_iq', 'mean_abs_id', 'mean_abs_iq', 'abc_sum_peak'});

fprintf('\n=== FOC Interface Case Diagnosis ===\n');
disp(summary);

fprintf('\nQuick interpretation:\n');
fprintf('  nominal                -> ideal torque current lands on +q\n');
fprintf('  theta_minus_pi_over_2  -> ideal torque current lands on -d\n');
fprintf('  theta_plus_pi_over_2   -> ideal torque current lands on +d\n');
fprintf('  current_neg            -> ideal torque current lands on -q\n');
fprintf('  theta/phase mismatch   -> dq energy spreads across both axes\n');

if opts.Plot
    plot_case_grid(waveforms);
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

function plot_case_grid(waveforms)
names = fieldnames(waveforms);
figure('Name', 'FOC Interface Diagnosis', 'Position', [100 100 1400 900]);
for idx = 1:numel(names)
    data = waveforms.(names{idx});
    subplot(4, 2, idx);
    plot(data.theta, data.id, data.theta, data.iq, 'LineWidth', 1.0);
    grid on;
    title(strrep(names{idx}, '_', '\_'));
    xlabel('theta_e [rad]');
    ylabel('Current [A]');
    legend('i_d', 'i_q', 'Location', 'best');
end
end