function results = run_transform_module_test()
%RUN_TRANSFORM_MODULE_TEST Module-level test for Clarke/Park mapping only.
%
% This test is independent of Simscape plant and PI loops. It verifies
% whether abc->alpha/beta->dq mapping keeps d-axis near zero for a balanced
% sinusoidal current set under candidate theta mapping modes.

Ipk = 8.0;
f_elec = 100;              % Electrical frequency [Hz]
Ts = 1e-5;
t_end = 0.05;
t = (0:Ts:t_end).';

theta_true = 2*pi*f_elec*t;
ia = Ipk * sin(theta_true);
ib = Ipk * sin(theta_true - 2*pi/3);
ic = Ipk * sin(theta_true + 2*pi/3);

modes = {'p_times_theta_m', 'theta_m', 'neg_p_times_theta_m', 'neg_theta_m'};
pp_candidates = [1, 4];

results = struct('mode', {}, 'pole_pairs', {}, 'id_rms', {}, 'iq_mean', {});

fprintf('=== Transform Module Test ===\n');
for i = 1:numel(modes)
    for k = 1:numel(pp_candidates)
        mode = modes{i};
        pp = pp_candidates(k);
        theta_m = theta_true / pp;
        theta_e = map_theta(mode, theta_m, pp);

        [id, iq] = abc_to_dq(ia, ib, ic, theta_e);
        id_rms = sqrt(mean(id.^2));
        iq_mean = mean(iq);

        results(end + 1) = struct( ...
            'mode', mode, ...
            'pole_pairs', pp, ...
            'id_rms', id_rms, ...
            'iq_mean', iq_mean); %#ok<AGROW>

        fprintf('mode=%-18s pp=%d  id_rms=%.5f  iq_mean=%.5f\n', mode, pp, id_rms, iq_mean);
    end
end

[~, idx] = min([results.id_rms]);
best = results(idx);

fprintf('\nBest mapping: mode=%s, pp=%d (id_rms=%.5f, iq_mean=%.5f)\n', ...
    best.mode, best.pole_pairs, best.id_rms, best.iq_mean);
end

function theta_e = map_theta(mode, theta_m, pp)
    switch lower(mode)
        case 'p_times_theta_m'
            theta_e = pp * theta_m;
        case 'theta_m'
            theta_e = theta_m;
        case 'neg_p_times_theta_m'
            theta_e = -pp * theta_m;
        case 'neg_theta_m'
            theta_e = -theta_m;
        otherwise
            error('Unsupported mode: %s', mode);
    end
end

function [id, iq] = abc_to_dq(ia, ib, ic, theta_e)
    i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
    i_beta  = (sqrt(3)/3) * (ib - ic);

    c = cos(theta_e);
    s = sin(theta_e);
    id = i_alpha .* c + i_beta .* s;
    iq = -i_alpha .* s + i_beta .* c;
end
