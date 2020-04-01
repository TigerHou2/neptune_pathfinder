% get_LPI.m 
%   returns rotation matrix from inertial to planet-relative frame
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
%   rot_angle - double(1), rad, angle rotated through since epoch
%   
% Output:
%   L_PI - double(3,3), nd, rotation matrix from the inertial frame to the
%                           planet relative frame

function [L_PI] = get_LPI(rot_angle)
%#codegen

L_PI = [ cos(rot_angle) sin(rot_angle) 0; ...
    -sin(rot_angle) cos(rot_angle) 0; ...
    0              0 1];


end % get_LPI