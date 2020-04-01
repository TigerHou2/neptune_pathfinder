% aa_default_in.m 
%   Create input structure with default values.
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
%   none
%   
% Outputs:
%   in - struct, multi, input structure with default values

function in = aa_default_in()
%#codegen

%% Define input structure
in = aa_define_in;


%% Constants

in.c.earth_g = 9.81;


%% Simulation data

% Termination conditions

% OR conditions
ii = 1;
in.s.term.or.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.or.var(ii) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(ii) = 0; % double, m, terminate at zero altitude
ii = ii + 1;

in.s.term.or.type(ii) = int8(1); % int8, nd, inequality - greater than
in.s.term.or.var(ii) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(ii) = 200e3; % double, m, terminate at 200km altitude
ii = ii + 1;

in.s.term.or.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.or.var(ii) = uint8(2); % uint8, nd, termination mode (mass)
in.s.term.or.value(ii) = 0; % double, kg, terminate at 0 kg mass
ii = ii + 1;

% AND conditions
% None currently.

% Trajectory
in.s.traj.rate = 5; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 1000; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [6503000; 0; 0]; % double(3), m, initial PCI position vector, 125 km altitude
in.s.traj.v_pci_ini = [-500; 7500; 0]; % double(3), m/s, initial PCI inertial velocity vector

% Other
in.s.data_rate = 1; % double, Hz, data logging rate


%% Planetary data

in.p = get_planetary_data( 2, 1 ); % Load settings for Earth, exponential atmopshere model


%% Vehicle data

in.v.mp.m_ini = 1000; % double, kg, vehicle mass

% Aerodynamics
in.v.aero.mode = uint8(1); % uint8, nd, aerodynamics mode
in.v.aero.aoa = 0; % double, rad, angle-of-attack
in.v.aero.cl = 0.3; % double, nd, lift coefficient
in.v.aero.cd = 1.4; % double, nd, drag coefficient
in.v.aero.area_ref = 1; % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1; % double, m, nose radius
temp = load(['.' filesep 'data' filesep 'aero_apollo.mat']);
in.v.aero.table = temp.table;

% GNC
% Guidance
in.v.gnc.g.p.rate = 0.5; % double, Hz, guidance rate
in.v.gnc.g.p.bank_mode = uint8(1); % uint8, nd, guidance mode - constant bank
in.v.gnc.g.p.const_bank = double(0); % double(0)
in.v.gnc.g.p.const_bank_rate = double(0.2); % double(0)
in.v.gnc.g.p.bank_schedule = zeros(100,2); % double(100,2)
% Control
in.v.gnc.c.p.rate = 5; % Hz, control rate
in.v.gnc.c.p.bank_mode = uint8(1); % nd, bank angle control mode, use perfect bank control
in.v.gnc.c.p.bank_rate_lim = double(0.3491); % rad/s, bank rate limit (20 deg/s)
in.v.gnc.c.p.bank_accel_lim = double(0); % rad/s^2, not currently used


end % aa_default_in()