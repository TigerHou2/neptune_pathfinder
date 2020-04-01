% skew.m 
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
%   vec - double(3), nd, vector from which the skew symmetric matrix is mad 
%     
% Output:
%   n - double(3,3), nd, skew symmetric matrix

function n = skew(vec) 
%#codegen

n = [     0  -vec(3)  vec(2); ...
      vec(3)      0  -vec(1); ...
     -vec(2)  vec(1)      0];

end % skew()

