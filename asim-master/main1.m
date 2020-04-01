% main.m 
%   Top-level asim function; returns output data structure based on
%   input data structure
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
%   in - struct, multi, Main input structure; see documentation
%     
% Output:
%   dat - struct, multi, Main output structure; see documentation

function dat = main1( in ) %#codegen


%% Initialize

[dat, veh] = initialize(in);


%% Run Trajectory

[dat] = trajectory(in,veh,dat);


end % main()