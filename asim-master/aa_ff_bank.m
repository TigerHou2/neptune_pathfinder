%% aa_ff_bank
%
% Author:   Tiger Hou
% Created:  02/21/2020
% Modified: 02/21/2020
%
% DESCRIPTION
%   - This script simulates aerocapture at Neptune using bank angle control
%       Variables include L/D, V0, EFPA, and Ap Tolerance
%   - For each configuration, two trajectories are flown (fully lift-down
%       and fully lift-up) to find the maximum boundaries.
%
% SOURCES:
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies
%   - [3] Neptune Aerocapture Systems Analysis
%

%% Initialization

aa_ff_setup
in = aa_ff_bank_in();    % load the input file


%% Run Simulation

% orbiting body gravitational parameter
mu = in.p.mu;

% FreeFlyer variables are assigned to the structure "in"
% Required variables:
%   - LoD [1x1][nd]
%   - fpa_init [1x1][deg]
%   - v_init [1x3][m/s]
%   - r_init [1x3][m/s]

in.v.aero.cd = in.v.aero.cl / LoD;
in.s.traj.v_pci_ini = norm(v_init) .* [sind(fpa_init); cosd(fpa_init); 0];
in.v.gnc.g.p_bank_a.y_f = 1;

% define the transformation matrix between FreeFlyer frame and ASIM frame
T = [r_init/norm(r_init); ...
     v_init/norm(v_init); ...
     cross(r_init',v_init')' / norm(cross(r_init',v_init')')];

% simulate
data = aa_main_mex(in);
    
% grab r and v data, return to FreeFlyer
idx = sum(~isnan(data.traj.time)); % termination index
r_init = data.traj.pos_ii(idx,:)';      % termination position
v_init = data.traj.vel_ii(idx,:)';      % termination velocity

% reverse transformation from ASIM to FreeFlyer
r_init = (T\r_init)';
v_init = (T\v_init)';