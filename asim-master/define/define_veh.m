% define_veh.m 
%   Define vehicle data structure
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
%   none
%   
% Output:
%   veh - struct(1), multi, vehicle data structure

function veh = define_veh()
%#codegen

%% Parameter data structure
p = struct( ...
    'm_jettison', double(0) );

%% State data structure
s = struct( ...
    'not_jettisoned', logical(true), ...
    'compute_trim', uint8(0), ...
    'area_ref', double(0), ...
    'aoa', double(0), ...
    'ssa', double(0), ...
    'bank', double(0) );

%% Construct combined input data structure
veh = struct( ...
    's', s, ...
    'p', p );

end % define_veh()