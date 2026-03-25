function results = run_plantcore_openloop_test(plant_model, opts)
%RUN_PLANTCORE_OPENLOOP_TEST Standalone open-loop test for blue plant wrapper.
%
% Usage:
%   run_plantcore_openloop_test('my_blue_plant_wrapper')
%   run_plantcore_openloop_test('my_blue_plant_wrapper', struct('t_end', 0.08))
%
% The script bypasses FOC and drives wrapper input gates_6 directly with
% open-loop SPWM gates. It is intended to verify PlantCore wiring and motor
% electrical response independently.

if nargin < 1 || isempty(plant_model)
    plant_model = strtrim(getenv('PMSM_SIMSCAPE_PLANT_MODEL'));
end
if isempty(plant_model)
    error('Missing plant model name.');
end
if nargin < 2
    opts = struct();
end

% Defaults
cfg = struct();
cfg.t_end = 0.08;
cfg.fsw = 20e3;
cfg.f_elec = 80;
cfg.mod_index = 0.6;
cfg.Te_load = 0.0;
cfg.time_step = 2e-6;

cfg = merge_struct(cfg, opts);

if cfg.mod_index <= 0 || cfg.mod_index >= 1
    error('mod_index should be in (0,1).');
end

fprintf('=== PlantCore Open-Loop Test ===\n');
fprintf('Plant model: %s\n', plant_model);

if ~bdIsLoaded(plant_model)
    load_system(plant_model);
end

[t, gates6, Te_load] = build_openloop_inputs(cfg);

in = Simulink.SimulationInput(plant_model);
in = in.setModelParameter('StopTime', num2str(cfg.t_end));
in = in.setModelParameter('SaveOutput', 'on');
in = in.setModelParameter('OutputSaveName', 'yout');
in = in.setModelParameter('SaveFormat', 'StructureWithTime');

in = in.setExternalInput([t, gates6, Te_load]);

sim_out = sim(in);
yout = sim_out.get('yout');

ia = yout.signals(1).values;
ib = yout.signals(2).values;
ic = yout.signals(3).values;
omega_m = yout.signals(4).values;
Te = yout.signals(5).values;

i_rms = [rms(ia), rms(ib), rms(ic)];
i_mean = [mean(ia), mean(ib), mean(ic)];
neutral_sum_rms = rms(ia + ib + ic);
i_std = [std(ia), std(ib), std(ic)];

results = struct();
results.t = yout.time;
results.ia = ia;
results.ib = ib;
results.ic = ic;
results.omega_m = omega_m;
results.Te = Te;
results.i_rms = i_rms;
results.i_mean = i_mean;
results.i_std = i_std;
results.neutral_sum_rms = neutral_sum_rms;

fprintf('i_rms           = [%.4f %.4f %.4f] A\n', i_rms);
fprintf('i_mean          = [%.4f %.4f %.4f] A\n', i_mean);
fprintf('neutral_sum_rms = %.4f A\n', neutral_sum_rms);

if max(i_std) < 1e-3
    fprintf('WARNING: Current response is near zero/constant. Check PlantCore wiring and gate mapping.\n');
end
if neutral_sum_rms > 0.5 * max(i_rms)
    fprintf('WARNING: ia+ib+ic is large; phase mapping or current measurement mapping may be wrong.\n');
end
end

function [t, gates6, Te_load] = build_openloop_inputs(cfg)
% Build 6-gate SPWM sequence: [Sa+ Sa- Sb+ Sb- Sc+ Sc-]

t = (0:cfg.time_step:cfg.t_end).';
theta = 2*pi*cfg.f_elec*t;

ma = 0.5 + 0.5*cfg.mod_index*sin(theta);
mb = 0.5 + 0.5*cfg.mod_index*sin(theta - 2*pi/3);
mc = 0.5 + 0.5*cfg.mod_index*sin(theta + 2*pi/3);

carrier = mod(t*cfg.fsw, 1.0);

Sa_p = double(ma >= carrier);
Sb_p = double(mb >= carrier);
Sc_p = double(mc >= carrier);

Sa_n = 1 - Sa_p;
Sb_n = 1 - Sb_p;
Sc_n = 1 - Sc_p;

gates6 = [Sa_p, Sa_n, Sb_p, Sb_n, Sc_p, Sc_n];
Te_load = cfg.Te_load * ones(size(t));
end

function dst = merge_struct(dst, src)
fields = fieldnames(src);
for i = 1:numel(fields)
    name = fields{i};
    dst.(name) = src.(name);
end
end
