function [did_dt, diq_dt, domega_dt, dtheta_dt] = pmsm_dq_model(vd, vq, id, iq, omega_m, Te_load, params)
%PMSM_DQ_MODEL PMSM dynamic model in dq reference frame
%
%   State equations for a surface-mount PMSM (Ld = Lq):
%
%   did/dt = (1/Ld) * (vd - Rs*id + omega_e*Lq*iq)
%   diq/dt = (1/Lq) * (vq - Rs*iq - omega_e*Ld*id - omega_e*flux_pm)
%   domega_m/dt = (1/J) * (Te - B*omega_m - Te_load)
%   dtheta_m/dt = omega_m
%
%   Electromagnetic torque:
%   Te = (3/2) * p * (flux_pm*iq + (Ld - Lq)*id*iq)
%
%   Inputs:
%       vd, vq      - d,q axis voltages [V]
%       id, iq      - d,q axis currents [A]
%       omega_m     - Mechanical speed [rad/s]
%       Te_load     - Load torque [N*m]
%       params      - struct with motor parameters
%   Outputs:
%       did_dt, diq_dt       - Current derivatives
%       domega_dt, dtheta_dt - Mechanical derivatives

    Rs = params.Rs;
    Ld = params.Ld;
    Lq = params.Lq;
    flux_pm = params.flux_pm;
    p = params.p;
    J = params.J;
    B = params.B;
    
    % Electrical angular velocity
    omega_e = p * omega_m;
    
    % Voltage equations (dq-frame)
    did_dt = (1/Ld) * (vd - Rs*id + omega_e*Lq*iq);
    diq_dt = (1/Lq) * (vq - Rs*iq - omega_e*Ld*id - omega_e*flux_pm);
    
    % Electromagnetic torque
    Te = 1.5 * p * (flux_pm*iq + (Ld - Lq)*id*iq);
    
    % Mechanical equation
    domega_dt = (1/J) * (Te - B*omega_m - Te_load);
    dtheta_dt = omega_m;
end
