function [v_alpha, v_beta] = inv_park_transform(vd, vq, theta_e)
%INV_PARK_TRANSFORM Inverse Park (dq -> alpha-beta) transform
%
%   [v_alpha, v_beta] = inv_park_transform(vd, vq, theta_e)
%
%   Implements the inverse Park transformation:
%   | v_alpha |   | cos(theta_e)  -sin(theta_e) | | vd |
%   | v_beta  | = | sin(theta_e)   cos(theta_e) | | vq |
%
%   Inputs:
%       vd, vq  - Rotating (dq) frame voltages [V]
%       theta_e - Electrical angle [rad]
%   Outputs:
%       v_alpha, v_beta - Stationary frame voltages [V]

    v_alpha = vd * cos(theta_e) - vq * sin(theta_e);
    v_beta  = vd * sin(theta_e) + vq * cos(theta_e);
end
