% main.m 
%   Top-level asim function; returns output data structure based on
%   input data structure
%
% Author:   Tiger Hou
% Created:  02/09/2020
% Modified: 02/18/2020
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

function dat = aa_main( in ) %#codegen


%% Initialize

[dat, veh] = aa_initialize(in);


%% Run Trajectory

[dat] = aa_trajectory(in,veh,dat);


end % aa_main()