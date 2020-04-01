% check_end_conditions.m 
%   Check simulation termination conditions
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
%   in - data structure, misc, input data structure
%   dat - data structure, misc, current vehicle trajectory data
%   calcs - data strucutre, misc, current vehicle misc. calculations
%   i - int8(1), nd, current length of the state vector 
%   
% Output:
%   flag - int8(1), nd, vehicle drag coefficient at angle of attack

function [flag] = check_end_conditions(in,calcs_curr,calcs_prev)
%#codegen

% Assign variables
flag = 0;

% Determine which side end trigger is during previous and current time step
switch in.s.term.mode
    case 1 % Altitude
        sign1 = sign(calcs_prev.alt - in.s.term.value);
        sign2 = sign(calcs_curr.alt - in.s.term.value);
    case 2 % Mach
        sign1 = sign(calcs_prev.mach - in.s.term.value);
        sign2 = sign(calcs_curr.mach - in.s.term.value);
    otherwise
        sign1 = nan;
        sign2 = nan;
end

if sign1 + sign2 == 0
    % Value has been crossed, determine direction
    if (in.s.term.direction == 1) && (sign2 == 1)
        flag = 1;
    elseif (in.s.term.direction == -1) && (sign2 == -1)
        flag = 1;
    elseif in.s.term.direction == 0
        flag = 1;
    end
end

% Determine if vehicle mass is below zero
if calcs_curr.mass < 0
    flag = 1;
end

end % check_end_conditions()
