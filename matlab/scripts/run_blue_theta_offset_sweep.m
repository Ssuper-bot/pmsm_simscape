function results = run_blue_theta_offset_sweep(simscape_plant_model, theta_e_mode)
%RUN_BLUE_THETA_OFFSET_SWEEP Sweep theta offset and compare id/iq behavior.
%
% Usage:
%   run_blue_theta_offset_sweep('my_blue_plant_wrapper')
%   run_blue_theta_offset_sweep('my_blue_plant_wrapper', 'p_times_theta_m')

if nargin < 1 || isempty(simscape_plant_model)
    simscape_plant_model = strtrim(getenv('PMSM_SIMSCAPE_PLANT_MODEL'));
end
if isempty(simscape_plant_model)
    error('Missing simscape_plant_model.');
end
if nargin < 2 || isempty(theta_e_mode)
    theta_e_mode = 'p_times_theta_m';
end

offsets = [-pi, -2*pi/3, -pi/2, -pi/3, -pi/6, 0, pi/6, pi/3, pi/2, 2*pi/3, pi];

results = struct('theta_e_offset', {}, 'id_rms', {}, 'iq_rms', {}, 'status', {}, 'message', {});

fprintf('=== Blue Theta Offset Sweep ===\n');
fprintf('Plant model: %s\n', simscape_plant_model);
fprintf('theta_e_mode: %s\n', theta_e_mode);

for i = 1:numel(offsets)
    off = offsets(i);
    fprintf('\n[%d/%d] theta_e_offset=%.6f rad\n', i, numel(offsets), off);

    try
        overrides = struct();
        overrides.run_sim = true;
        overrides.ref_params = struct( ...
            'speed_ref', 600, ...
            'speed_ramp_time', 0.08, ...
            'load_torque', 0.0, ...
            'load_step_time', 0.3, ...
            'id_ref', 0.0);
        overrides.sim_params = struct( ...
            't_end', 0.12, ...
            'controller_mode', 'pid_sfun', ...
            'plant_mode', 'simscape_blue', ...
            'simscape_plant_input', 'gates_6', ...
            'simscape_plant_model', simscape_plant_model, ...
            'theta_e_mode', theta_e_mode, ...
            'theta_e_offset', off, ...
            'enable_debug_logging', true, ...
            'current_noise_std', 0.0, ...
            'theta_noise_std', 0.0, ...
            'current_meas_lpf_hz', 2000.0, ...
            'omega_meas_lpf_hz', 500.0);

        [~, sim_out] = pmsm_foc_simscape(overrides);

        id_ts = get_ts(sim_out, 'id_meas_ts');
        iq_ts = get_ts(sim_out, 'iq_meas_ts');
        ia_ts = get_ts(sim_out, 'ia_meas_ts');

        t0 = 0.04;
        id_rms = rms_after_t(id_ts, t0);
        iq_rms = rms_after_t(iq_ts, t0);
        ia_std = std_after_t(ia_ts, t0);

        if ia_std < 1e-4
            results(end + 1) = struct( ...
                'theta_e_offset', off, ...
                'id_rms', id_rms, ...
                'iq_rms', iq_rms, ...
                'status', 'FAIL', ...
                'message', 'Non-informative run: ia_meas is near zero. Check PlantCore connection/model name.'); %#ok<AGROW>
            fprintf('  FAIL: ia_meas near zero, result is non-informative\n');
            continue;
        end

        results(end + 1) = struct( ...
            'theta_e_offset', off, ...
            'id_rms', id_rms, ...
            'iq_rms', iq_rms, ...
            'status', 'PASS', ...
            'message', 'ok'); %#ok<AGROW>

        fprintf('  id_rms=%.4f, iq_rms=%.4f\n', id_rms, iq_rms);
    catch ME
        results(end + 1) = struct( ...
            'theta_e_offset', off, ...
            'id_rms', NaN, ...
            'iq_rms', NaN, ...
            'status', 'FAIL', ...
            'message', ME.message); %#ok<AGROW>

        fprintf('  FAIL: %s\n', ME.message);
    end
end

pass_idx = find(strcmp({results.status}, 'PASS'));
if isempty(pass_idx)
    fprintf('\nNo successful run in sweep.\n');
    return;
end

id_vals = [results(pass_idx).id_rms];
[best_id, k] = min(id_vals);
best_off = results(pass_idx(k)).theta_e_offset;

fprintf('\nRecommended theta_e_offset (minimum id_rms): %.6f rad (id_rms=%.4f)\n', best_off, best_id);
end

function ts = get_ts(sim_out, name)
    ts = [];
    try
        ts = sim_out.get(name);
    catch
    end

    if isempty(ts)
        ts = evalin('base', name);
    end

    if ~isa(ts, 'timeseries')
        error('Signal %s is not available as timeseries.', name);
    end
end

function v = rms_after_t(ts, t0)
    idx = ts.Time >= t0;
    if ~any(idx)
        error('No samples found after t0=%.4f s.', t0);
    end
    data = ts.Data(idx);
    v = sqrt(mean(data.^2));
end

function v = std_after_t(ts, t0)
    idx = ts.Time >= t0;
    if ~any(idx)
        error('No samples found after t0=%.4f s.', t0);
    end
    data = ts.Data(idx);
    v = std(data);
end
