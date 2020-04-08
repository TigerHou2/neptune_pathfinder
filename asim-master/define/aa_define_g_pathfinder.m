% aa_define_g_pathfinder.m 
%   Define pathfinder data collection probe data structure
%
% Author:   Tiger Hou
% Created:  03/03/2020
% Modified: 03/03/2020
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
%   g_pathfinder - struct(1), multi, empty guidance data structure

function [g_pathfinder] = aa_define_g_pathfinder()
%#codegen


%% Parameter data structure
  
p = struct( ... 
    'mass', double(0), ...  % kg, spacecraft initial mass
    'area_ref', double(0), ... % m^2, drag
    'tgt_ap', double(0), ...% m, target apoapsis
    'tgt_pe', double(0), ...% m, target periapsis
    'cl', double(0), ...    % nd, lift coefficient
    'cd', double(0), ...    % nd, drag coefficient
    'omega', double(zeros(3,1)), ... % rad/s, planet rotation rate
    'type', double(1), ... % nd, int, whether vehicle is probe(1) or main(2)
    'scale', double(zeros(3,1)), ... % nd, 3-sigma scaling error of r,v,a
    'bias', double(zeros(9,1)), ...  % m, m/s, m/s^2, 3-sigma bias of r,v,a
    'tilt', double(zeros(3,1)) ...   % rad, 3-sigma axis misalignment of r,v,a w.r.t. x-axis
    );  


%% States data structure

s = struct( ...
    'cmd_bank', double(0), ...
    'fpa', double(0), ...
    'rho_est', double(0), ...
    't', double(0), ... % s, current time
    't_ini', double(0), ... % s, initialization time
    'R_pci', double(zeros(3,1)), ... % m, current position vector
    'V_inrtl_pci', double(zeros(3,1)), ... % m/s, current inertial velocity vector
    'V_pf_pci', double(zeros(3,1)), ... % m/s, current planet-relative, planet-fixed velocity vector
    'A_sens_pci', double(zeros(3,1)) ... % m/s^2, sensed acceleration magnitude
    );


%% States data structure

i = struct( ...
    't', double(0), ... % s, current time
    'R_pci', double(zeros(3,1)), ... % m, current position vector
    'V_inrtl_pci', double(zeros(3,1)), ... % m/s, current inertial velocity vector
    'V_pf_pci', double(zeros(3,1)), ... % m/s, current planet-relative, planet-fixed velocity vector
    'A_sens_pci', double(zeros(3,1)) ... % m/s^2, sensed acceleration magnitude
    ); 
    


%% Construct combined input data structure

g_pathfinder = struct( ...
    'i', i, ...    
    's', s, ...
    'p', p ...
    );

end % aa_define_g_pathfinder()
