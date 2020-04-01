% oe2rv.m
%   Convert orbital elements to inertial position and velocity vectors
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
%   mu - double(1), gravitational parameter for attracting body
%   oe - double(6x1), misc, vector of orbital elements
%     oe(1) = semi-major axis (sma)
%     oe(2) = eccentricity magnitude (ecc)
%     oe(3) = inclination (inc)
%     oe(4) = argument of periapsis (apr)
%     oe(5) = longitude of ascending node (lan)
%     oe(6) = true anomaly angle (taa)
%
% Output:
%   rv - double(6x1), multi, rv vector, i.e. [R;V]

function [rv]=oe2rv(oe,mu)

sma = oe(1);
ecc = oe(2);
inc = oe(3);
apr = oe(4);
lan = oe(5);
taa = oe(6);

slr = sma * (1 - ecc^2);
r = slr / (1 + ecc*cos(taa));

r_PQW = [r*cos(taa); r*sin(taa); 0];
v_PQW = [-sqrt(mu/slr)*sin(taa); sqrt(mu/slr)*(ecc + cos(taa)); 0];

% form rotation matrix
R3_m_lan = [ cos(-lan) sin(-lan) 0;
            -sin(-lan) cos(-lan) 0;
             0         0         1];
R1_m_inc = [1 0          0;
            0 cos(-inc)  sin(-inc);
            0 -sin(-inc) cos(-inc)];
R3_m_apr = [ cos(-apr) sin(-apr) 0;
            -sin(-apr) cos(-apr) 0;
             0         0         1];

R = R3_m_lan * R1_m_inc * R3_m_apr;
         
r_IJK = R * r_PQW;
v_IJK = R * v_PQW;

rv = [r_IJK; v_IJK];

end