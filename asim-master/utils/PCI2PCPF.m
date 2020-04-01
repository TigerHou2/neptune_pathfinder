% PCI2PCPF.m 
%   Vector transformation between inertial frame and planet-centric frame
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
%   vec_ii - double(3), misc, vector in inertial frame
%   t0 - double(1), s, epoch time
%   t - double(1), s, time since epoch
%   omega - double(3), rad/s, planet's rotation rate
%   
% Output:
%   vec_pp - double(3), misc, vector in planet-centric frame

function [vec_pp] = PCI2PCPF(vec_ii, omega, t0, t) %codegen

% Inertial to planet relative transformation
rot_angle = norm(omega)*(t+t0); % Rotation angle [rad]
[L_PI] = get_LPI(rot_angle);

% Perform the vector transformation
vec_pp = L_PI*vec_ii; % Position vector planet/planet [m]

end % PCI2PCPF()