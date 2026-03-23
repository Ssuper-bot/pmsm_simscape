function [id, iq] = park_transform(i_alpha, i_beta, theta_e)
%PARK_TRANSFORM Park (alpha-beta -> dq) transform
%
%   [id, iq] = park_transform(i_alpha, i_beta, theta_e)
%
%   Implements the Park transformation:
%   | id |   |  cos(theta_e)  sin(theta_e) | | i_alpha |
%   | iq | = | -sin(theta_e)  cos(theta_e) | | i_beta  |
%
%   Inputs:
%       i_alpha, i_beta - Stationary frame currents [A]
%       theta_e         - Electrical angle [rad]
%   Outputs:
%       id, iq - Rotating (dq) frame currents [A]

    id =  i_alpha * cos(theta_e) + i_beta * sin(theta_e);
    iq = -i_alpha * sin(theta_e) + i_beta * cos(theta_e);
end
