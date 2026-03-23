%% Test PMSM Library Functions
% Quick smoke tests for Clarke, Park, SVPWM, and PMSM model.

clear; clc;
fprintf('Running PMSM Library Tests...\n');

%% Test Clarke Transform
[alpha, beta] = pmsm_lib.clarke_transform(1, -0.5, -0.5);
assert(abs(alpha - 1.0) < 1e-10, 'Clarke alpha failed');
assert(abs(beta) < 1e-10, 'Clarke beta failed');
fprintf('  Clarke Transform: PASS\n');

%% Test Park Transform
[d, q] = pmsm_lib.park_transform(1, 0, 0);
assert(abs(d - 1.0) < 1e-10, 'Park d failed');
assert(abs(q) < 1e-10, 'Park q failed');
fprintf('  Park Transform: PASS\n');

%% Test Inverse Park Transform
vd = 3; vq = 4; theta = 0.7;
[va, vb] = pmsm_lib.inv_park_transform(vd, vq, theta);
[d2, q2] = pmsm_lib.park_transform(va, vb, theta);
assert(abs(d2 - vd) < 1e-10, 'Inv Park d failed');
assert(abs(q2 - vq) < 1e-10, 'Inv Park q failed');
fprintf('  Inverse Park Transform: PASS\n');

%% Test SVPWM (zero voltage -> 50% duty)
[da, db, dc] = pmsm_lib.svpwm_modulator(0, 0, 24);
assert(abs(da - 0.5) < 1e-10, 'SVPWM da failed');
assert(abs(db - 0.5) < 1e-10, 'SVPWM db failed');
assert(abs(dc - 0.5) < 1e-10, 'SVPWM dc failed');
fprintf('  SVPWM Modulator: PASS\n');

%% Test PMSM dq Model
params.Rs = 0.5; params.Ld = 1.4e-3; params.Lq = 1.4e-3;
params.flux_pm = 0.0577; params.p = 4; params.J = 1.74e-5; params.B = 1e-4;
[did, diq, domega, dtheta] = pmsm_lib.pmsm_dq_model(0, 0, 0, 0, 0, 0, params);
assert(did == 0, 'PMSM did failed');
assert(diq == 0, 'PMSM diq failed');
fprintf('  PMSM dq Model: PASS\n');

fprintf('All MATLAB tests PASSED.\n');
