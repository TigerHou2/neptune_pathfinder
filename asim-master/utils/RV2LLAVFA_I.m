% RV2LLAVFA_I.m
%   Transformation between state vector components and inertial vectors
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
%   pos_ii - double(3), m, position in inertial frame
%   vel_ii - double(3), m/s, velocity in inertial frame
%   omega - double(3), rad/s, angular velocity vector of planet
%   t0 - double(1), s, epoch time
%   t - double(1), s, current time
%   r_e - double(1), m, equatorial radius
%   r_p - double(1), m, polar radius
%   
% Output:
%   lat - double(1), rad, geodetic latitude
%   lon - double(1), rad, longitude
%   alt - double(1), m, altitude above the ellipsoid
%   gamma_pp - double(1), rad, flight-path angle (negative downward)
%   az - double(1), rad, azimuth angle
%   vel_pp_mag - double(1), m/s, planet-centric velocity magnitude (speed)

function [lat, lon, alt, gamma_pp, az, vel_pp_mag] = ...
    RV2LLAVFA_I( pos_ii, vel_ii, omega, t0, t, r_e, r_p)
%codegen

% Inertial to planet relative transformation matrix
rot_angle = norm(omega)*(t+t0); % Rotation angle [rad]
[L_PI] = get_LPI(rot_angle);

% Inertial to planet relative transformation
pos_pp = L_PI * pos_ii;
vel_pp = L_PI * (vel_ii - cross(omega,pos_ii));

% Get Components
[lat, lon, alt, gamma_pp, az, vel_pp_mag] = ...
    RV2LLAVFA_P( pos_pp, vel_pp, r_e, r_p ); 

end % RV2LLAVFA_I()