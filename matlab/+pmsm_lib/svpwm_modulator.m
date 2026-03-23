function [da, db, dc] = svpwm_modulator(v_alpha, v_beta, Vdc)
%SVPWM_MODULATOR Space Vector PWM modulator
%
%   [da, db, dc] = svpwm_modulator(v_alpha, v_beta, Vdc)
%
%   Computes three-phase duty cycles using Space Vector PWM.
%   Uses the min-max injection method (equivalent to centered SVPWM).
%
%   Inputs:
%       v_alpha, v_beta - Stationary frame voltages [V]
%       Vdc             - DC bus voltage [V]
%   Outputs:
%       da, db, dc - Duty cycles for phases a, b, c [0, 1]

    % Inverse Clarke to get three-phase reference voltages
    va = v_alpha;
    vb = -0.5 * v_alpha + sqrt(3)/2 * v_beta;
    vc = -0.5 * v_alpha - sqrt(3)/2 * v_beta;
    
    % Normalize by Vdc/2
    va_n = va / (Vdc / 2);
    vb_n = vb / (Vdc / 2);
    vc_n = vc / (Vdc / 2);
    
    % Min-max injection (centered SVPWM)
    vmax = max([va_n, vb_n, vc_n]);
    vmin = min([va_n, vb_n, vc_n]);
    v_offset = -(vmax + vmin) / 2;
    
    % Apply offset and convert to duty cycle [0, 1]
    da = (va_n + v_offset + 1) / 2;
    db = (vb_n + v_offset + 1) / 2;
    dc = (vc_n + v_offset + 1) / 2;
    
    % Clamp to [0, 1]
    da = max(0, min(1, da));
    db = max(0, min(1, db));
    dc = max(0, min(1, dc));
end
