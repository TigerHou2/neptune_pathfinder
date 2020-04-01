% unit.m 
%   Compute unit vector
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
%   
% Output:
%   u - double(3), nd, unit vector along input, a


function u = unit(a)

    a_mag = sqrt( dot(a,a) );
    u = a ./ a_mag;

end % unit