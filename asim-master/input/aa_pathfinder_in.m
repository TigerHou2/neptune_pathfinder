%% aa_pathfinder_in
%   Input file for pathfinder data collection probe simulation
% 
% Author:   Tiger Hou
% Created:  02/29/2020
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
%   in - struct, multi, input structure with parameters for pathfinder
%   guidance

%%
function in = aa_pathfinder_in()
%#codegen

%% Define input structure
in = aa_default_in;


%% Planetary data

in.p = get_planetary_data( 8, 2 ); % Load settings for Neptune, table atm model


%% Simulation data

% Termination conditions
in.s.term.or.type(1) = int8(0); % int8, nd, crossing/equality condition
in.s.term.or.var(1) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(1) = 500e3; % double, m, terminate at atmospheric exit
in.s.term.or.direction(1) = int8(1); % int8, nd, positive crossing

in.s.term.or.type(2) = int8(-1); % int8, nd, crossing/equality condition
in.s.term.or.var(2) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(2) = 0.0; % double, m, terminate at atmospheric exit

% Trajectory
v0 = 29e3; % m/s
fpa0 = -7.0; % deg
alt0 = 500e3; % m

in.s.traj.rate = 100; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 2500; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [(in.p.r_e + alt0); 0; 0]; % double(3), m, initial PCI position vector
in.s.traj.v_pci_ini = v0 .* [sind(fpa0); cosd(fpa0); 0];  % Inertial velocity

in.s.data_rate = 1;

%% Vehicle data

in.v.mp.m_ini = 3300; % double, kg, vehicle mass

% Navigation
in.v.gnc.n.p.mode = uint8(2); % Use Markov process/ECRV error model
in.v.gnc.n.p.rate = 20; % Hz
in.v.gnc.n.p.seed = uint32(1); % nd, Error model seed
in.v.gnc.n.p.tau = 600; % s, time constant
in.v.gnc.n.p.omega = in.p.omega; % rad/s, planet angular velocity vector
in.v.gnc.n.p.r_e = in.p.r_e; % m, planet equatorial radius
in.v.gnc.n.p.r_p = in.p.r_p; % m, planet polar radius

% Aerodynamics
LoD = 0;

in.v.aero.mode = uint8(1); % uint8, nd, constant aerodynamic coefficients
in.v.aero.cd = 1.4;
in.v.aero.cl = in.v.aero.cd * LoD;
in.v.aero.area_ref = pi*(4.5/2)^2; % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1; % double, m, nose radius

% GNC
in.v.gnc.g.p.bank_mode = uint8(6); % uint8, nd, pathfinder probe/main guidance

% Guidance parameters
in.v.gnc.g.p.const_bank = 0; % double, rad, constant bank angle


end % aa_pathfinder_in()
