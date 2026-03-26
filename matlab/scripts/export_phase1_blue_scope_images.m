function export_phase1_blue_scope_images(simscape_plant_model, simscape_plant_input, out_dir)
%EXPORT_PHASE1_BLUE_SCOPE_IMAGES Run blue-plant baseline cases and export plots.
%   export_phase1_blue_scope_images('my_blue_plant_wrapper', 'gates_6')

if nargin < 1 || isempty(simscape_plant_model)
    simscape_plant_model = 'my_blue_plant_wrapper';
end
if nargin < 2 || isempty(simscape_plant_input)
    simscape_plant_input = 'gates_6';
end
if nargin < 3 || isempty(out_dir)
    out_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'attachment');
end
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

project_root = fullfile(fileparts(mfilename('fullpath')), '..', '..');
cd(project_root);
setup_project;

common_overrides = struct();
common_overrides.sim_params = struct( ...
    'controller_mode', 'pid_sfun', ...
    'plant_mode', 'simscape_blue', ...
    'simscape_plant_model', simscape_plant_model, ...
    'simscape_plant_input', simscape_plant_input, ...
    'enable_debug_logging', true, ...
    'run_sim', true);

cases = {
    struct('name', 'blue_speed_step_only', ...
           'ref_params', struct('load_torque', 0.0, 'speed_ref', 1000, 'load_step_time', 0.3)),
    struct('name', 'blue_speed_and_load_step', ...
           'ref_params', struct('load_torque', 0.1, 'speed_ref', 1000, 'load_step_time', 0.3))
};

for i = 1:numel(cases)
    c = cases{i};
    overrides = common_overrides;
    overrides.ref_params = c.ref_params;

    [~, sim_out] = pmsm_foc_simscape(overrides);

    ia = sim_out.get('ia_meas_ts');
    ib = sim_out.get('ib_meas_ts');
    ic = sim_out.get('ic_meas_ts');
    id = sim_out.get('id_meas_ts');
    iq = sim_out.get('iq_meas_ts');
    om = sim_out.get('omega_m_ts');
    duty = sim_out.get('duty_abc_ts');

    has_gates = any(strcmp(sim_out.who, 'gates_6_ts'));
    if has_gates
        gates = sim_out.get('gates_6_ts');
    else
        gates = [];
    end

    f1 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(ia.Time, ia.Data, 'LineWidth', 1.0); hold on;
    plot(ib.Time, ib.Data, 'LineWidth', 1.0);
    plot(ic.Time, ic.Data, 'LineWidth', 1.0); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - Blue Plant Phase Currents ia/ib/ic', c.name), 'Interpreter', 'none');
    legend('ia', 'ib', 'ic', 'Location', 'best');
    exportgraphics(f1, fullfile(out_dir, [c.name '_phase_currents.png']), 'Resolution', 160);
    close(f1);

    f2 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(id.Time, id.Data, 'LineWidth', 1.2); hold on;
    plot(iq.Time, iq.Data, 'LineWidth', 1.2); hold off;
    grid on;
    xlabel('Time (s)');
    ylabel('Current (A)');
    title(sprintf('%s - Blue Plant d/q Currents', c.name), 'Interpreter', 'none');
    legend('id_{meas}', 'iq_{meas}', 'Location', 'best');
    exportgraphics(f2, fullfile(out_dir, [c.name '_dq_currents.png']), 'Resolution', 160);
    close(f2);

    f3 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    plot(om.Time, om.Data, 'LineWidth', 1.2);
    grid on;
    xlabel('Time (s)');
    ylabel('Speed (rad/s)');
    title(sprintf('%s - Blue Plant Mechanical Speed', c.name), 'Interpreter', 'none');
    exportgraphics(f3, fullfile(out_dir, [c.name '_omega_m.png']), 'Resolution', 160);
    close(f3);

    f4 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 720]);
    dd = duty.Data;
    if isvector(dd)
        plot(duty.Time, dd, 'LineWidth', 1.0);
        legend('duty', 'Location', 'best');
    else
        plot(duty.Time, dd(:, 1), 'LineWidth', 1.0); hold on;
        plot(duty.Time, dd(:, 2), 'LineWidth', 1.0);
        plot(duty.Time, dd(:, 3), 'LineWidth', 1.0); hold off;
        legend('d_a', 'd_b', 'd_c', 'Location', 'best');
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Duty');
    ylim([0 1]);
    title(sprintf('%s - Blue Plant Duty Commands', c.name), 'Interpreter', 'none');
    exportgraphics(f4, fullfile(out_dir, [c.name '_duty_abc.png']), 'Resolution', 160);
    close(f4);

    if has_gates
        f5 = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1400 800]);
        gd = gates.Data;
        plot(gates.Time, gd(:, 1), 'LineWidth', 0.8); hold on;
        plot(gates.Time, gd(:, 2), 'LineWidth', 0.8);
        plot(gates.Time, gd(:, 3), 'LineWidth', 0.8);
        plot(gates.Time, gd(:, 4), 'LineWidth', 0.8);
        plot(gates.Time, gd(:, 5), 'LineWidth', 0.8);
        plot(gates.Time, gd(:, 6), 'LineWidth', 0.8); hold off;
        grid on;
        xlabel('Time (s)');
        ylabel('Gate State');
        ylim([-0.1 1.1]);
        title(sprintf('%s - Blue Plant gates_6', c.name), 'Interpreter', 'none');
        legend('Sa+', 'Sa-', 'Sb+', 'Sb-', 'Sc+', 'Sc-', 'Location', 'eastoutside');
        exportgraphics(f5, fullfile(out_dir, [c.name '_gates_6.png']), 'Resolution', 160);
        close(f5);
    end
end

fprintf('Blue plant export complete. Output dir: %s\n', out_dir);
end