function out = pmsm_model_fcn(Va, Vb, Vc, Te_load, id, iq, omega_m, theta_m, Rs, Ld, Lq, flux_pm, pp, J, B)
%PMSM_MODEL_FCN PMSM dq-frame model for MATLAB Fcn block
%   Inputs:
%     Va, Vb, Vc   - three-phase voltages
%     Te_load      - load torque
%     id, iq       - dq-axis currents (from integrator feedback)
%     omega_m      - mechanical speed (from integrator feedback)
%     theta_m      - mechanical angle (from integrator feedback)
%     Rs, Ld, Lq, flux_pm, pp, J, B - motor parameters (baked-in)
%
%   Output: [did_dt; diq_dt; dwm_dt; dtheta_dt; ia; ib; ic; Te]

    % Electrical angle and speed
    theta_e = pp * theta_m;
    we = pp * omega_m;

    % Clarke transform (abc -> alpha-beta)
    v_alpha = 2/3 * (Va - 0.5*Vb - 0.5*Vc);
    v_beta  = 2/3 * (sqrt(3)/2*Vb - sqrt(3)/2*Vc);

    % Park transform (alpha-beta -> dq)
    vd =  v_alpha*cos(theta_e) + v_beta*sin(theta_e);
    vq = -v_alpha*sin(theta_e) + v_beta*cos(theta_e);

    % dq voltage equations
    did_dt = (1/Ld) * (vd - Rs*id + we*Lq*iq);
    diq_dt = (1/Lq) * (vq - Rs*iq - we*Ld*id - we*flux_pm);

    % Electromagnetic torque
    Te = 1.5 * pp * (flux_pm * iq + (Ld - Lq) * id * iq);

    % Mechanical dynamics
    dwm_dt = (1/J) * (Te - B*omega_m - Te_load);
    dtheta_dt = omega_m;

    % Inverse Park (dq -> alpha-beta) for currents
    i_alpha = id*cos(theta_e) - iq*sin(theta_e);
    i_beta  = id*sin(theta_e) + iq*cos(theta_e);

    % Inverse Clarke (alpha-beta -> abc) for currents
    ia = i_alpha;
    ib = -0.5*i_alpha + sqrt(3)/2*i_beta;
    ic = -0.5*i_alpha - sqrt(3)/2*i_beta;

    out = [did_dt; diq_dt; dwm_dt; dtheta_dt; ia; ib; ic; Te];
end
