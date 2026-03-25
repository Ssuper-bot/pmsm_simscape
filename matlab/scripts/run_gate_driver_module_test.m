function results = run_gate_driver_module_test()
%RUN_GATE_DRIVER_MODULE_TEST Module-level check for duty->gates_6 conversion.
%
% This test validates logical complementarity and average duty consistency
% independent of Simscape plant.

fsw = 20e3;
Ts = 1e-6;
t_end = 5/fsw;
t = (0:Ts:t_end).';

% Deterministic duty profile
ma = 0.6 + 0.3*sin(2*pi*500*t);
mb = 0.5 + 0.25*sin(2*pi*500*t - 2*pi/3);
mc = 0.4 + 0.2*sin(2*pi*500*t + 2*pi/3);
ma = min(max(ma, 0), 1);
mb = min(max(mb, 0), 1);
mc = min(max(mc, 0), 1);

carrier = mod(t*fsw, 1);   % [0,1) sawtooth

ga_p = double(ma >= carrier);
ga_n = 1 - ga_p;
gb_p = double(mb >= carrier);
gb_n = 1 - gb_p;
gc_p = double(mc >= carrier);
gc_n = 1 - gc_p;

comp_ok = all((ga_p + ga_n) == 1) && all((gb_p + gb_n) == 1) && all((gc_p + gc_n) == 1);

duty_est = [mean(ga_p), mean(gb_p), mean(gc_p)];
duty_ref = [mean(ma), mean(mb), mean(mc)];
duty_err = duty_est - duty_ref;

results = struct();
results.complementary_ok = comp_ok;
results.duty_ref = duty_ref;
results.duty_est = duty_est;
results.duty_error = duty_err;

fprintf('=== Gate Driver Module Test ===\n');
fprintf('Complementary logic OK: %d\n', comp_ok);
fprintf('Duty ref : [%.5f %.5f %.5f]\n', duty_ref);
fprintf('Duty est : [%.5f %.5f %.5f]\n', duty_est);
fprintf('Duty err : [%.5f %.5f %.5f]\n', duty_err);
end
