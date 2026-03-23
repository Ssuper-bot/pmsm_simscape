function duty = svpwm_inline(v_alpha, v_beta, Vdc)
%SVPWM_INLINE Space Vector PWM - inline version for MATLAB Fcn block
%   Returns [da; db; dc] as a 3x1 vector

    % Inverse Clarke: alpha-beta -> abc
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

    % Duty cycles [0, 1]
    da = max(0, min(1, (va_n + v_offset + 1) / 2));
    db = max(0, min(1, (vb_n + v_offset + 1) / 2));
    dc = max(0, min(1, (vc_n + v_offset + 1) / 2));

    duty = [da; db; dc];
end
