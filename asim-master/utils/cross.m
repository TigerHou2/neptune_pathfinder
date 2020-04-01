% cross.m 
%   This function overrides the intrinsic Matlab cross function for speed.
%
% Aeroassist Simulation (asim) Source Code
%
% Original version developed by M. J. Grant, Z. R. Putnam
%   Space Systems Design Lab
%   Georgia Institute of Technology
%
% Maintained by Putnam Research Group
%   Department of Aerospace Engineering
%   University of Illinois at Urbana-Champaign
%
% Input:
%   a - double(3), nd, vector
%   b - double(3), nd, vector
%   
% Output:
%   c - double(3), nd, cross product of a and b

function c = cross(a,b)
%#codegen

% Compute cross product using skew-symmetric matrix
c = skew(a)*b;

end % cross()