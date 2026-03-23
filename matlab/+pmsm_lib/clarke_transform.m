function [i_alpha, i_beta] = clarke_transform(ia, ib, ic)
%CLARKE_TRANSFORM Power-invariant Clarke (abc -> alpha-beta) transform
%
%   [i_alpha, i_beta] = clarke_transform(ia, ib, ic)
%
%   Implements the power-invariant Clarke transformation:
%   | i_alpha |       | 1   -1/2    -1/2   | | ia |
%   | i_beta  | = 2/3 | 0  sqrt(3)/2 -sqrt(3)/2 | | ib |
%                                                    | ic |
%
%   Inputs:
%       ia, ib, ic - Three-phase currents [A]
%   Outputs:
%       i_alpha, i_beta - Stationary frame currents [A]

    k = 2/3;
    i_alpha = k * (ia - 0.5*ib - 0.5*ic);
    i_beta  = k * (sqrt(3)/2 * ib - sqrt(3)/2 * ic);
end
