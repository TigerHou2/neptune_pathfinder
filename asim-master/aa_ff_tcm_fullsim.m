%% aa_ff_tcm_fullsim
%
% Author:   Tiger Hou
% Created:  02/13/2020
% Modified: 02/13/2020
%
% DESCRIPTION
%   - This script simulates aerocapture at Neptune using drag modulation.
%       It interfaces with FreeFlyer by taking the SC state at EI and
%       then uses ASIM to perform aerocapture. It then hands the SC state
%       back to FreeFlyer to perform orbit calculations / visualizations.
%
% SOURCES:
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies
%   - [3] Neptune Aerocapture Systems Analysis
%

%% Initialization

setup
in = aa_ff_tcm_fullsim_in();    % load the input file

% create Neptune atmosphere dispersion model
% (currently we are not using it)
% nep_atm = aa_make_nep_atm(in);


%% Run Simulation

% orbiting body gravitational parameter
mu = in.p.mu;

% FreeFlyer variables are assigned to the structure "in"
% Required variables:
%   - beta_ratio [1x1][nd]
%   - fpa_init [1x1][deg]
%   - v_init [1x3][m/s]
%   - r_init [1x3][m/s]
beta1 = 10;
beta2 = beta1 * beta_ratio;
v0 = norm(v_init);
fpa0 = fpa_init;
% define the transformation matrix between FreeFlyer frame and ASIM frame
T = [r_init/norm(r_init); ...
     v_init/norm(v_init); ...
     cross(r_init',v_init')' / norm(cross(r_init',v_init')')];

% modify variables that are affected by FreeFlyer inputs
in.v.gnc.g.p_sej_a.area_ref = ...
        [in.v.aero.area_ref; in.v.mp.m_ini/(in.v.aero.cd*beta2)]; % m^2
in.s.traj.v_pci_ini = v0*[sind(fpa0); cosd(fpa0); 0];

% simulate
data = aa_main_mex(in);

% grab r and v data, return to FreeFlyer
idx = sum(~isnan(data.traj.time)); % termination index
r_init = data.traj.pos_ii(idx,:)';      % termination position
v_init = data.traj.vel_ii(idx,:)';      % termination velocity

% reverse transformation from ASIM to FreeFlyer
r_init = (T\r_init)';
v_init = (T\v_init)';
