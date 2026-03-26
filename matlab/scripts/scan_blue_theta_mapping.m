function [mode_results, offset_results] = scan_blue_theta_mapping(simscape_plant_model, out_dir)
%SCAN_BLUE_THETA_MAPPING Scan theta_e mode and offset for blue-plant setup.
%
% Phase A: scan theta_e_mode with offset=0
% Phase B: scan theta_e_offset with best mode from phase A

if nargin < 1 || isempty(simscape_plant_model)
    simscape_plant_model = 'my_blue_plant_wrapper';
end
if nargin < 2 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'attachment', 'theta_scan');
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

project_root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(project_root);
setup_project;

% Imported PMSMDrive example blocks reference these base-workspace symbols.
assignin('base', 'Rs', 0.5);
assignin('base', 'Ld', 1.4e-3);
assignin('base', 'Lq', 1.4e-3);
assignin('base', 'PM', 0.0577);
assignin('base', 'N', 4);
assignin('base', 'J', 1.74e-5);
assignin('base', 'Ts', 1 / 20e3);

best_gate_order = [1 3 5 2 4 6];

mode_candidates = {'p_times_theta_m', 'theta_m', 'neg_p_times_theta_m', 'neg_theta_m'};
mode_results = table('Size', [numel(mode_candidates) 9], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'theta_e_mode','theta_e_offset','phase_mean_spread','phase_sum_abs_mean','id_mean_abs','iq_mean_abs','omega_mean','ia_rms','score'});

for i = 1:numel(mode_candidates)
    mode = mode_candidates{i};
    tag = ['mode_' mode];
    stats = run_one_case(simscape_plant_model, best_gate_order, mode, 0.0, out_dir, tag);

    mode_results.theta_e_mode(i) = string(mode);
    mode_results.theta_e_offset(i) = 0.0;
    mode_results.phase_mean_spread(i) = stats.phase_mean_spread;
    mode_results.phase_sum_abs_mean(i) = stats.phase_sum_abs_mean;
    mode_results.id_mean_abs(i) = stats.id_mean_abs;
    mode_results.iq_mean_abs(i) = stats.iq_mean_abs;
    mode_results.omega_mean(i) = stats.omega_mean;
    mode_results.ia_rms(i) = stats.ia_rms;
    mode_results.score(i) = stats.score;
end

mode_results = sortrows(mode_results, 'score', 'ascend');
writetable(mode_results, fullfile(out_dir, 'theta_mode_scan_ranked.csv'));

best_mode = char(mode_results.theta_e_mode(1));

offset_candidates = [-pi, -pi/2, -pi/3, -pi/6, 0, pi/6, pi/3, pi/2, pi];
offset_results = table('Size', [numel(offset_candidates) 9], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'theta_e_mode','theta_e_offset','phase_mean_spread','phase_sum_abs_mean','id_mean_abs','iq_mean_abs','omega_mean','ia_rms','score'});

for i = 1:numel(offset_candidates)
    off = offset_candidates(i);
    off_tag = sprintf('offset_%+0.4f', off);
    off_tag = strrep(off_tag, '.', 'p');
    off_tag = strrep(off_tag, '+', 'pos');
    off_tag = strrep(off_tag, '-', 'neg');

    tag = ['offsetscan_' off_tag];
    stats = run_one_case(simscape_plant_model, best_gate_order, best_mode, off, out_dir, tag);

    offset_results.theta_e_mode(i) = string(best_mode);
    offset_results.theta_e_offset(i) = off;
    offset_results.phase_mean_spread(i) = stats.phase_mean_spread;
    offset_results.phase_sum_abs_mean(i) = stats.phase_sum_abs_mean;
    offset_results.id_mean_abs(i) = stats.id_mean_abs;
    offset_results.iq_mean_abs(i) = stats.iq_mean_abs;
    offset_results.omega_mean(i) = stats.omega_mean;
    offset_results.ia_rms(i) = stats.ia_rms;
    offset_results.score(i) = stats.score;
end

offset_results = sortrows(offset_results, 'score', 'ascend');
writetable(offset_results, fullfile(out_dir, 'theta_offset_scan_ranked.csv'));

fprintf('Theta scan complete. out_dir=%s\n', out_dir);
fprintf('Best mode: %s\n', best_mode);
fprintf('Best offset (rad): %.6f\n', offset_results.theta_e_offset(1));
end

function stats = run_one_case(simscape_plant_model, gates_order, theta_mode, theta_offset, out_dir, tag)
overrides = struct();
overrides.ref_params = struct( ...
    'speed_ref', 1000, ...
    'speed_ramp_time', 0.1, ...
    'load_torque', 0.0, ...
    'load_step_time', 0.3, ...
    'id_ref', 0);
overrides.sim_params = struct( ...
    't_end', 0.5, ...
    'controller_mode', 'pid_sfun', ...
    'plant_mode', 'simscape_blue', ...
    'simscape_plant_input', 'gates_6', ...
    'simscape_plant_model', simscape_plant_model, ...
    'enable_debug_logging', true, ...
    'gates_order', gates_order, ...
    'theta_e_mode', theta_mode, ...
    'theta_e_offset', theta_offset, ...
    'run_sim', true);

[~, sim_out] = pmsm_foc_simscape(overrides);

ia = sim_out.get('ia_meas_ts');
ib = sim_out.get('ib_meas_ts');
ic = sim_out.get('ic_meas_ts');
id = sim_out.get('id_meas_ts');
iq = sim_out.get('iq_meas_ts');
om = sim_out.get('omega_m_ts');

ia_d = ia.Data;
ib_d = ib.Data;
ic_d = ic.Data;
id_d = id.Data;
iq_d = iq.Data;
om_d = om.Data;

stats.phase_mean_spread = max([mean(ia_d), mean(ib_d), mean(ic_d)]) - min([mean(ia_d), mean(ib_d), mean(ic_d)]);
stats.phase_sum_abs_mean = mean(abs(ia_d + ib_d + ic_d));
stats.id_mean_abs = abs(mean(id_d));
stats.iq_mean_abs = abs(mean(iq_d));
stats.omega_mean = mean(om_d);
stats.ia_rms = rms(ia_d);

stats.score = stats.phase_mean_spread + 0.5 * stats.phase_sum_abs_mean + stats.id_mean_abs - 0.1 * stats.omega_mean;

f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
plot(ia.Time, ia_d, 'LineWidth', 1.0); hold on;
plot(ib.Time, ib_d, 'LineWidth', 1.0);
plot(ic.Time, ic_d, 'LineWidth', 1.0); hold off;
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('%s - phase currents', tag), 'Interpreter', 'none');
legend('ia', 'ib', 'ic', 'Location', 'best');
exportgraphics(f1, fullfile(out_dir, [tag '_phase_currents.png']), 'Resolution', 140);
close(f1);

f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
plot(id.Time, id_d, 'LineWidth', 1.0); hold on;
plot(iq.Time, iq_d, 'LineWidth', 1.0); hold off;
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('%s - dq currents', tag), 'Interpreter', 'none');
legend('id', 'iq', 'Location', 'best');
exportgraphics(f2, fullfile(out_dir, [tag '_dq_currents.png']), 'Resolution', 140);
close(f2);
end
