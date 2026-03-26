function export_blue_final_tuned_images(simscape_plant_model, out_dir)
%EXPORT_BLUE_FINAL_TUNED_IMAGES Export tuned blue-plant waveforms for two cases.

if nargin < 1 || isempty(simscape_plant_model)
    simscape_plant_model = 'my_blue_plant_wrapper';
end
if nargin < 2 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'attachment', 'final_tuned');
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

project_root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(project_root);
setup_project;

% Required by borrowed example blocks in PlantCore
assignin('base', 'Rs', 0.5);
assignin('base', 'Ld', 1.4e-3);
assignin('base', 'Lq', 1.4e-3);
assignin('base', 'PM', 0.0577);
assignin('base', 'N', 4);
assignin('base', 'J', 1.74e-5);
assignin('base', 'Ts', 1 / 20e3);

best_gate_order = [1 3 5 2 4 6];
best_theta_mode = 'neg_p_times_theta_m';
best_theta_offset = -pi / 6;

cases = {
    struct('name', 'tuned_speed_step_only', 'load_torque', 0.0), ...
    struct('name', 'tuned_speed_and_load_step', 'load_torque', 0.1)
};

rows = strings(numel(cases)+1, 1);
rows(1) = "case,mean_ia,mean_ib,mean_ic,mean_id,mean_iq,mean_omega";

for i = 1:numel(cases)
    c = cases{i};

    overrides = struct();
    overrides.ref_params = struct( ...
        'speed_ref', 1000, ...
        'speed_ramp_time', 0.1, ...
        'load_torque', c.load_torque, ...
        'load_step_time', 0.3, ...
        'id_ref', 0);
    overrides.sim_params = struct( ...
        't_end', 0.5, ...
        'controller_mode', 'pid_sfun', ...
        'plant_mode', 'simscape_blue', ...
        'simscape_plant_input', 'gates_6', ...
        'simscape_plant_model', simscape_plant_model, ...
        'enable_debug_logging', true, ...
        'gates_order', best_gate_order, ...
        'theta_e_mode', best_theta_mode, ...
        'theta_e_offset', best_theta_offset, ...
        'run_sim', true);

    [~, sim_out] = pmsm_foc_simscape(overrides);

    ia = sim_out.get('ia_meas_ts');
    ib = sim_out.get('ib_meas_ts');
    ic = sim_out.get('ic_meas_ts');
    id = sim_out.get('id_meas_ts');
    iq = sim_out.get('iq_meas_ts');
    om = sim_out.get('omega_m_ts');

    rows(i+1) = sprintf('%s,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g', c.name, ...
        mean(ia.Data), mean(ib.Data), mean(ic.Data), mean(id.Data), mean(iq.Data), mean(om.Data));

    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(ia.Time, ia.Data, 'LineWidth', 1.0); hold on;
    plot(ib.Time, ib.Data, 'LineWidth', 1.0);
    plot(ic.Time, ic.Data, 'LineWidth', 1.0); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - phase currents', c.name), 'Interpreter', 'none');
    legend('ia', 'ib', 'ic', 'Location', 'best');
    exportgraphics(f1, fullfile(out_dir, [c.name '_phase_currents.png']), 'Resolution', 160);
    close(f1);

    f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(id.Time, id.Data, 'LineWidth', 1.0); hold on;
    plot(iq.Time, iq.Data, 'LineWidth', 1.0); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - dq currents', c.name), 'Interpreter', 'none');
    legend('id', 'iq', 'Location', 'best');
    exportgraphics(f2, fullfile(out_dir, [c.name '_dq_currents.png']), 'Resolution', 160);
    close(f2);

    f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(om.Time, om.Data, 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('omega_m (rad/s)');
    title(sprintf('%s - omega_m', c.name), 'Interpreter', 'none');
    exportgraphics(f3, fullfile(out_dir, [c.name '_omega_m.png']), 'Resolution', 160);
    close(f3);
end

fid = fopen(fullfile(out_dir, 'tuned_summary.csv'), 'w');
for i = 1:numel(rows)
    fprintf(fid, '%s\n', rows(i));
end
fclose(fid);

fprintf('Tuned export complete: %s\n', out_dir);
end
