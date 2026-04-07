function compare_first_order_bandwidths()
%COMPARE_FIRST_ORDER_BANDWIDTHS Compare exact-cancel PI, mismatched PI, and P-only.
%   The plant is the normalized RL current model
%       Gp_n(s) = omega_e / (s + omega_e),   omega_e = R / L
%   and the UI compares three reference-to-output closed loops:
%       1) exact pole-zero cancellation PI
%       2) mismatched PI zero placement
%       3) P-only control

defaults = struct(...
    'omega_e', 350.0, ...
    'omega_cl', 1200.0, ...
    'omega_in', 300.0, ...
    'zero_ratio', 0.35, ...
    'step_amp', 1.0, ...
    'ramp_slope', 4.0);

state = defaults;

fig = uifigure('Name', 'PI Zero Cancellation Comparison', ...
    'Position', [80 80 1500 920], ...
    'Color', [0.98 0.98 0.97]);

main_grid = uigridlayout(fig, [2 2]);
main_grid.ColumnWidth = {380, '1x'};
main_grid.RowHeight = {'1x', 190};
main_grid.Padding = [16 16 16 16];
main_grid.RowSpacing = 12;
main_grid.ColumnSpacing = 12;

control_panel = uipanel(main_grid, 'Title', 'Controls');
control_panel.Layout.Row = [1 2];
control_panel.Layout.Column = 1;
control_panel.Scrollable = 'on';

control_grid = uigridlayout(control_panel, [20 1]);
control_grid.RowHeight = {22, 54, 22, 22, 54, 22, 22, 54, 22, 22, 54, 34, 22, 32, 22, 32, '1x', 40, 40, 8};
control_grid.Padding = [12 12 12 12];
control_grid.RowSpacing = 8;

omega_e_title = ui_label(control_grid, 'Plant electrical pole  omega_e = R/L  [rad/s]');
omega_e_title.Layout.Row = 1;
omega_e_slider = ui_slider(control_grid, log10(10), log10(5000), state.omega_e, @on_slider_changing, @on_slider_changed);
omega_e_slider.Layout.Row = 2;
omega_e_value = ui_value_label(control_grid, '');
omega_e_value.Layout.Row = 3;

omega_cl_title = ui_label(control_grid, 'Target closed-loop bandwidth  omega_cl  [rad/s]');
omega_cl_title.Layout.Row = 4;
omega_cl_slider = ui_slider(control_grid, log10(10), log10(8000), state.omega_cl, @on_slider_changing, @on_slider_changed);
omega_cl_slider.Layout.Row = 5;
omega_cl_value = ui_value_label(control_grid, '');
omega_cl_value.Layout.Row = 6;

omega_in_title = ui_label(control_grid, 'Reference sinusoid frequency  omega_in  [rad/s]');
omega_in_title.Layout.Row = 7;
omega_in_slider = ui_slider(control_grid, log10(1), log10(8000), state.omega_in, @on_slider_changing, @on_slider_changed);
omega_in_slider.Layout.Row = 8;
omega_in_value = ui_value_label(control_grid, '');
omega_in_value.Layout.Row = 9;

zero_ratio_title = ui_label(control_grid, 'Mismatched PI zero ratio  gamma = (Ki/Kp) / omega_e');
zero_ratio_title.Layout.Row = 10;
zero_ratio_slider = ui_slider(control_grid, log10(0.1), log10(3.0), state.zero_ratio, @on_slider_changing, @on_slider_changed);
zero_ratio_slider.Layout.Row = 11;
zero_ratio_value = ui_value_label(control_grid, '');
zero_ratio_value.Layout.Row = 12;

step_amp_label = ui_label(control_grid, 'Step amplitude');
step_amp_label.Layout.Row = 13;
step_amp_field = uieditfield(control_grid, 'numeric', ...
    'Value', state.step_amp, ...
    'Limits', [0.1 20.0], ...
    'RoundFractionalValues', false, ...
    'ValueDisplayFormat', '%.3g', ...
    'ValueChangedFcn', @on_numeric_changed);
step_amp_field.Layout.Row = 14;

ramp_slope_label = ui_label(control_grid, 'Ramp slope');
ramp_slope_label.Layout.Row = 15;
ramp_slope_field = uieditfield(control_grid, 'numeric', ...
    'Value', state.ramp_slope, ...
    'Limits', [0.01 100.0], ...
    'RoundFractionalValues', false, ...
    'ValueDisplayFormat', '%.3g', ...
    'ValueChangedFcn', @on_numeric_changed);
ramp_slope_field.Layout.Row = 16;

formula_text = uitextarea(control_grid, ...
    'Editable', 'off', ...
    'FontName', 'Menlo', ...
    'FontSize', 12, ...
    'Value', {
        'Plant and controllers:'
        'Gp_n(s) = omega_e / (s + omega_e)'
        'Exact PI:      Kp = omega_cl/omega_e, Ki = omega_cl'
        'Mismatched PI: Kp = omega_cl/omega_e, Ki = gamma * omega_cl'
        'P-only:        Kp = omega_cl/omega_e, Ki = 0'
        ''
        'Exact cancel means Ki/Kp = omega_e.'
        'That places the controller zero at -omega_e.'
        'If gamma ~= 1, the zero misses the plant pole.'
        'Then the closed loop stays second-order.'
    });
formula_text.Layout.Row = 17;

tip1 = ui_label(control_grid, 'Tip: set gamma far from 1 to see overshoot, drag, or slower settling in the mismatched PI case.');
tip1.Layout.Row = 18;
tip2 = ui_label(control_grid, 'Tip: P-only can be fast but it keeps a steady-state tracking error for step commands.');
tip2.Layout.Row = 19;

plot_grid = uigridlayout(main_grid, [2 2]);
plot_grid.Layout.Row = 1;
plot_grid.Layout.Column = 2;
plot_grid.Padding = [0 0 0 0];
plot_grid.RowSpacing = 10;
plot_grid.ColumnSpacing = 10;

ax_step = uiaxes(plot_grid);
title(ax_step, 'Step Tracking  r -> y');
xlabel(ax_step, 'Time [s]');
ylabel(ax_step, 'Output');
grid(ax_step, 'on');

ax_sine = uiaxes(plot_grid);
title(ax_sine, 'Sinusoidal Tracking  r -> y');
xlabel(ax_sine, 'Time [s]');
ylabel(ax_sine, 'Output');
grid(ax_sine, 'on');

ax_ramp = uiaxes(plot_grid);
title(ax_ramp, 'Ramp Tracking  r -> y');
xlabel(ax_ramp, 'Time [s]');
ylabel(ax_ramp, 'Output');
grid(ax_ramp, 'on');

ax_freq = uiaxes(plot_grid);
title(ax_freq, 'Closed-Loop Frequency Response');
xlabel(ax_freq, 'Frequency [rad/s]');
grid(ax_freq, 'on');

summary_area = uitextarea(main_grid, ...
    'Editable', 'off', ...
    'FontName', 'Menlo', ...
    'FontSize', 12, ...
    'Visible', 'off');
summary_area.Layout.Row = 2;
summary_area.Layout.Column = 2;

omega_e_slider.UserData = 'omega_e';
omega_cl_slider.UserData = 'omega_cl';
omega_in_slider.UserData = 'omega_in';
zero_ratio_slider.UserData = 'zero_ratio';
step_amp_field.UserData = 'step_amp';
ramp_slope_field.UserData = 'ramp_slope';

update_view();

    function on_slider_changing(src, event)
        state.(src.UserData) = 10.^event.Value;
        update_view();
    end

    function on_slider_changed(src, event)
        state.(src.UserData) = 10.^event.Value;
        update_view();
    end

    function on_numeric_changed(src, ~)
        state.(src.UserData) = src.Value;
        update_view();
    end

    function update_view()
        set_pole_label(omega_e_value, state.omega_e);
        set_pole_label(omega_cl_value, state.omega_cl);
        set_pole_label(omega_in_value, state.omega_in);
        zero_ratio_value.Text = sprintf('gamma = %.3f, mismatched zero = -%.2f rad/s', ...
            state.zero_ratio, state.zero_ratio * state.omega_e);

        omega_max = max([state.omega_e, state.omega_cl, state.zero_ratio * state.omega_cl, state.omega_in, 1.0]);
        omega_min = max(min([state.omega_e, state.omega_cl, state.omega_in, 1.0]), 1e-6);
        t_end = max([8.0 / omega_min, 4.0 * 2.0 * pi / max(state.omega_in, 1e-6), 0.08]);
        dt = min([1.0 / (500.0 * omega_max), t_end / 6000.0]);
        dt = max(dt, 1e-6);
        t = 0:dt:t_end;

        ref_step = state.step_amp * ones(size(t));
        ref_sine = state.step_amp * sin(state.omega_in * t);
        ref_ramp = state.ramp_slope * t;

        kp = state.omega_cl / max(state.omega_e, 1e-12);
        exact_ctrl = struct('kp', kp, 'ki', state.omega_cl, 'name', 'exact PI');
        mismatch_ctrl = struct('kp', kp, 'ki', state.zero_ratio * state.omega_cl, 'name', 'mismatched PI');
        p_only_ctrl = struct('kp', kp, 'ki', 0.0, 'name', 'P-only');

        step_exact = simulate_closed_loop(state.omega_e, exact_ctrl.kp, exact_ctrl.ki, ref_step, dt);
        step_mismatch = simulate_closed_loop(state.omega_e, mismatch_ctrl.kp, mismatch_ctrl.ki, ref_step, dt);
        step_p_only = simulate_closed_loop(state.omega_e, p_only_ctrl.kp, p_only_ctrl.ki, ref_step, dt);

        sine_exact = simulate_closed_loop(state.omega_e, exact_ctrl.kp, exact_ctrl.ki, ref_sine, dt);
        sine_mismatch = simulate_closed_loop(state.omega_e, mismatch_ctrl.kp, mismatch_ctrl.ki, ref_sine, dt);
        sine_p_only = simulate_closed_loop(state.omega_e, p_only_ctrl.kp, p_only_ctrl.ki, ref_sine, dt);

        ramp_exact = simulate_closed_loop(state.omega_e, exact_ctrl.kp, exact_ctrl.ki, ref_ramp, dt);
        ramp_mismatch = simulate_closed_loop(state.omega_e, mismatch_ctrl.kp, mismatch_ctrl.ki, ref_ramp, dt);
        ramp_p_only = simulate_closed_loop(state.omega_e, p_only_ctrl.kp, p_only_ctrl.ki, ref_ramp, dt);

        exact_color = [0.10 0.45 0.80];
        mismatch_color = [0.84 0.33 0.16];
        p_only_color = [0.19 0.60 0.32];

        cla(ax_step);
        plot(ax_step, t, ref_step, ':', 'Color', [0.70 0.70 0.70], 'LineWidth', 1.2);
        hold(ax_step, 'on');
        plot(ax_step, t, step_exact, 'LineWidth', 1.8, 'Color', exact_color);
        plot(ax_step, t, step_mismatch, 'LineWidth', 1.8, 'Color', mismatch_color);
        plot(ax_step, t, step_p_only, 'LineWidth', 1.8, 'Color', p_only_color);
        hold(ax_step, 'off');
        legend(ax_step, {'reference', 'exact PI', 'mismatched PI', 'P-only'}, 'Location', 'southeast');

        cla(ax_sine);
        plot(ax_sine, t, ref_sine, ':', 'Color', [0.70 0.70 0.70], 'LineWidth', 1.2);
        hold(ax_sine, 'on');
        plot(ax_sine, t, sine_exact, 'LineWidth', 1.8, 'Color', exact_color);
        plot(ax_sine, t, sine_mismatch, 'LineWidth', 1.8, 'Color', mismatch_color);
        plot(ax_sine, t, sine_p_only, 'LineWidth', 1.8, 'Color', p_only_color);
        hold(ax_sine, 'off');
        legend(ax_sine, {'reference', 'exact PI', 'mismatched PI', 'P-only'}, 'Location', 'southeast');

        cla(ax_ramp);
        plot(ax_ramp, t, ref_ramp, ':', 'Color', [0.70 0.70 0.70], 'LineWidth', 1.2);
        hold(ax_ramp, 'on');
        plot(ax_ramp, t, ramp_exact, 'LineWidth', 1.8, 'Color', exact_color);
        plot(ax_ramp, t, ramp_mismatch, 'LineWidth', 1.8, 'Color', mismatch_color);
        plot(ax_ramp, t, ramp_p_only, 'LineWidth', 1.8, 'Color', p_only_color);
        hold(ax_ramp, 'off');
        legend(ax_ramp, {'reference', 'exact PI', 'mismatched PI', 'P-only'}, 'Location', 'northwest');

        w = logspace(log10(max(1e-1, omega_min / 20.0)), log10(max(10.0, omega_max * 20.0)), 600);
        resp_exact = closed_loop_resp(state.omega_e, exact_ctrl.kp, exact_ctrl.ki, w);
        resp_mismatch = closed_loop_resp(state.omega_e, mismatch_ctrl.kp, mismatch_ctrl.ki, w);
        resp_p_only = closed_loop_resp(state.omega_e, p_only_ctrl.kp, p_only_ctrl.ki, w);

        cla(ax_freq);
        yyaxis(ax_freq, 'left');
        semilogx(ax_freq, w, mag_db(resp_exact), 'LineWidth', 1.8, 'Color', exact_color);
        hold(ax_freq, 'on');
        semilogx(ax_freq, w, mag_db(resp_mismatch), 'LineWidth', 1.8, 'Color', mismatch_color);
        semilogx(ax_freq, w, mag_db(resp_p_only), 'LineWidth', 1.8, 'Color', p_only_color);
        xline(ax_freq, state.omega_in, ':', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.0);
        ylabel(ax_freq, 'Magnitude [dB]');

        yyaxis(ax_freq, 'right');
        semilogx(ax_freq, w, phase_deg(resp_exact), '--', 'LineWidth', 1.4, 'Color', exact_color);
        semilogx(ax_freq, w, phase_deg(resp_mismatch), '--', 'LineWidth', 1.4, 'Color', mismatch_color);
        semilogx(ax_freq, w, phase_deg(resp_p_only), '--', 'LineWidth', 1.4, 'Color', p_only_color);
        ylabel(ax_freq, 'Phase [deg]');
        hold(ax_freq, 'off');
        legend(ax_freq, {'|T_exact|', '|T_mismatch|', '|T_P|', 'omega_{in}', ...
            'phase exact', 'phase mismatch', 'phase P'}, 'Location', 'southwest');

        update_summary(summary_area, state, exact_ctrl, mismatch_ctrl, t, ...
            step_exact, step_mismatch, step_p_only, ref_ramp, ramp_exact, ramp_mismatch, ramp_p_only);

        fig.Name = sprintf('Plant pole %.1f rad/s, target closed loop %.1f rad/s, mismatch gamma %.2f', ...
            state.omega_e, state.omega_cl, state.zero_ratio);
    end
end

function lbl = ui_label(parent, text)
lbl = uilabel(parent, 'Text', text, 'WordWrap', 'on', 'FontSize', 13);
end

function lbl = ui_value_label(parent, text)
lbl = uilabel(parent, 'Text', text, 'WordWrap', 'off', 'FontSize', 12, 'FontColor', [0.25 0.25 0.25]);
end

function slider = ui_slider(parent, log_min, log_max, initial_value, changing_cb, changed_cb)
slider = uislider(parent, ...
    'Limits', [log_min, log_max], ...
    'Value', log10(initial_value), ...
    'MajorTicks', [], ...
    'MinorTicks', [], ...
    'ValueChangingFcn', changing_cb, ...
    'ValueChangedFcn', changed_cb);
end

function set_pole_label(lbl, value)
lbl.Text = sprintf('%.2f rad/s   tau = %.3f ms', value, 1e3 / value);
end

function y = simulate_closed_loop(omega_e, kp, ki, ref, dt)
y = zeros(size(ref));
integral_state = 0.0;
for idx = 2:numel(ref)
    error_value = ref(idx - 1) - y(idx - 1);
    control_effort = kp * error_value + ki * integral_state;
    y_dot = -omega_e * y(idx - 1) + omega_e * control_effort;
    y(idx) = y(idx - 1) + dt * y_dot;
    integral_state = integral_state + dt * error_value;
end
end

function resp = closed_loop_resp(omega_e, kp, ki, w)
s = 1i * w;
resp = omega_e * (kp * s + ki) ./ (s.^2 + omega_e * (1.0 + kp) * s + omega_e * ki);
end

function values = mag_db(resp)
values = 20.0 * log10(max(abs(resp), 1e-12));
end

function values = phase_deg(resp)
values = unwrap(angle(resp)) * 180.0 / pi;
end

function update_summary(summary_area, state, exact_ctrl, mismatch_ctrl, t, ...
    step_exact, step_mismatch, step_p_only, ref_ramp, ramp_exact, ramp_mismatch, ramp_p_only)

stats_exact = step_stats(t, step_exact);
stats_mismatch = step_stats(t, step_mismatch);
stats_p_only = step_stats(t, step_p_only);

ramp_err_exact = ref_ramp(end) - ramp_exact(end);
ramp_err_mismatch = ref_ramp(end) - ramp_mismatch(end);
ramp_err_p_only = ref_ramp(end) - ramp_p_only(end);

summary_area.Value = {
    sprintf('Plant pole:               -%.2f rad/s', state.omega_e)
    sprintf('Exact PI zero:            -%.2f rad/s', exact_ctrl.ki / max(exact_ctrl.kp, 1e-12))
    sprintf('Mismatched PI zero:       -%.2f rad/s', mismatch_ctrl.ki / max(mismatch_ctrl.kp, 1e-12))
    'P-only zero:              none'
    sprintf('Shared proportional gain: Kp = %.4f', exact_ctrl.kp)
    sprintf('Exact PI integral gain:   Ki = %.4f', exact_ctrl.ki)
    sprintf('Mismatch PI integral gain:Ki = %.4f', mismatch_ctrl.ki)
    ''
    sprintf('Exact PI rise/settle:     %.4f s / %.4f s', stats_exact.rise_time, stats_exact.settling_time)
    sprintf('Mismatch rise/settle:     %.4f s / %.4f s', stats_mismatch.rise_time, stats_mismatch.settling_time)
    sprintf('P-only rise/settle:       %.4f s / %.4f s', stats_p_only.rise_time, stats_p_only.settling_time)
    sprintf('Exact PI ramp error:      %.4f', ramp_err_exact)
    sprintf('Mismatch PI ramp error:   %.4f', ramp_err_mismatch)
    sprintf('P-only ramp error:        %.4f', ramp_err_p_only)
    ''
    'Interpretation:'
    '- Exact PI puts the controller zero on the plant pole, so the closed loop behaves like a clean first-order target.'
    '- Mismatched PI keeps a leftover second-order mode. The farther gamma is from 1, the less clean the response.'
    '- P-only has no zero cancellation and no integral action, so it keeps step tracking error and weaker low-frequency tracking.'
};
summary_area.Visible = 'on';
end

function stats = step_stats(t, y)
final_value = y(end);
if abs(final_value) < 1e-12
    stats = struct('rise_time', NaN, 'settling_time', NaN);
    return;
end

if final_value >= 0
    idx10 = find(y >= 0.1 * final_value, 1, 'first');
    idx90 = find(y >= 0.9 * final_value, 1, 'first');
else
    idx10 = find(y <= 0.1 * final_value, 1, 'first');
    idx90 = find(y <= 0.9 * final_value, 1, 'first');
end

if isempty(idx10) || isempty(idx90)
    rise_time = NaN;
else
    rise_time = t(idx90) - t(idx10);
end

band = 0.02 * abs(final_value);
settling_idx = find(abs(y - final_value) > band, 1, 'last');
if isempty(settling_idx) || settling_idx == numel(t)
    settling_time = 0.0;
else
    settling_time = t(settling_idx + 1);
end

stats = struct('rise_time', rise_time, 'settling_time', settling_time);
end