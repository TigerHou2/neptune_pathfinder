% get_alt.m 
%   return altitude based on current position vector
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
%   pos_pp - double(3), m, position vector in planet-centric frame
%   re - double(1), m, equatorial radius
%   rp - double(1), m, polar radius
%   lat - double(1), rad, latitude
%     
% Outputs:
%   alt - double(1), m, altitude above planetary ellipsoid

function alt = get_alt(pos_pp, re, rp, lat) 
%#codegen

f = (re-rp)/re; % flattening
e2 = 1-(1-f)^2; % ellipticity
s = norm([pos_pp(1) pos_pp(2)]);
N = re / sqrt(1-e2*sin(lat)^2); % radius of curvature in the vertical prime
alt = s*cos(lat) + (pos_pp(3) + e2*N*sin(lat))*sin(lat) - N;

end % get_alt()