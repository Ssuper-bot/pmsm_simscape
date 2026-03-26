function results = scan_blue_gate_orders(simscape_plant_model, out_dir)
%SCAN_BLUE_GATE_ORDERS Sweep candidate gates_6 mappings for blue plant.
%
% Outputs:
%   - PNG waveforms per candidate in out_dir
%   - CSV summary in out_dir/gate_order_scan_summary.csv

if nargin < 1 || isempty(simscape_plant_model)
    simscape_plant_model = 'my_blue_plant_wrapper';
end
if nargin < 2 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'attachment', 'gate_order_scan');
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

project_root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(project_root);
setup_project;

% Candidate mappings from gate source order
% Base source order is [Sa+, Sa-, Sb+, Sb-, Sc+, Sc-].
candidates = {
    struct('name', 'order_123456', 'order', [1 2 3 4 5 6]), ...
    struct('name', 'order_135246', 'order', [1 3 5 2 4 6]), ...
    struct('name', 'order_214365', 'order', [2 1 4 3 6 5]), ...
    struct('name', 'order_125634', 'order', [1 2 5 6 3 4]), ...
    struct('name', 'order_345612', 'order', [3 4 5 6 1 2]), ...
    struct('name', 'order_563412', 'order', [5 6 3 4 1 2])
};

results = table('Size', [numel(candidates) 10], ...
    'VariableTypes', {'string','string','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'case_name','order','phase_mean_spread','phase_sum_abs_mean','id_mean_abs','iq_mean_abs','omega_mean','ia_rms','ib_rms','ic_rms'});

for i = 1:numel(candidates)
    c = candidates{i};

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
        'gates_order', c.order, ...
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

    phase_mean_spread = max([mean(ia_d), mean(ib_d), mean(ic_d)]) - min([mean(ia_d), mean(ib_d), mean(ic_d)]);
    phase_sum_abs_mean = mean(abs(ia_d + ib_d + ic_d));

    results.case_name(i) = string(c.name);
    results.order(i) = string(sprintf('%d-%d-%d-%d-%d-%d', c.order));
    results.phase_mean_spread(i) = phase_mean_spread;
    results.phase_sum_abs_mean(i) = phase_sum_abs_mean;
    results.id_mean_abs(i) = abs(mean(id_d));
    results.iq_mean_abs(i) = abs(mean(iq_d));
    results.omega_mean(i) = mean(om_d);
    results.ia_rms(i) = rms(ia_d);
    results.ib_rms(i) = rms(ib_d);
    results.ic_rms(i) = rms(ic_d);

    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(ia.Time, ia_d, 'LineWidth', 1.0); hold on;
    plot(ib.Time, ib_d, 'LineWidth', 1.0);
    plot(ic.Time, ic_d, 'LineWidth', 1.0); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - Phase Currents', c.name), 'Interpreter', 'none');
    legend('ia', 'ib', 'ic', 'Location', 'best');
    exportgraphics(f1, fullfile(out_dir, [c.name '_phase_currents.png']), 'Resolution', 140);
    close(f1);

    f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(id.Time, id_d, 'LineWidth', 1.0); hold on;
    plot(iq.Time, iq_d, 'LineWidth', 1.0); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - dq Currents', c.name), 'Interpreter', 'none');
    legend('id', 'iq', 'Location', 'best');
    exportgraphics(f2, fullfile(out_dir, [c.name '_dq_currents.png']), 'Resolution', 140);
    close(f2);
end

writetable(results, fullfile(out_dir, 'gate_order_scan_summary.csv'));

% Heuristic ranking: lower spread/id bias/sum is better, higher speed is better.
score = results.phase_mean_spread + 0.5 * results.phase_sum_abs_mean + results.id_mean_abs - 0.1 * results.omega_mean;
results = addvars(results, score, 'NewVariableNames', 'score');
results = sortrows(results, 'score', 'ascend');
writetable(results, fullfile(out_dir, 'gate_order_scan_ranked.csv'));

disp(results);
fprintf('Gate order scan complete: %s\n', out_dir);
end
