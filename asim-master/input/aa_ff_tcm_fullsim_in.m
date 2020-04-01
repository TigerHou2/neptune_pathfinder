%% aa_ff_tcm_fullsim_in
%   First test file for using asim
% 
% Author:   Tiger Hou
% Created:  02/07/2020
% Modified: 02/17/2020
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
%   in - struct, multi, input structure with parameters for SEJ guidance

%%
function in = aa_ff_tcm_fullsim_in()
%#codegen

%% Define input structure
in = aa_default_in;


%% Planetary data

in.p = get_planetary_data( 8, 2 ); % Load settings for Earth, table atm model


%% Constants

in.c.earth_g = 9.81;


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
in.s.traj.t_max = 3000; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [(in.p.r_e + alt0); 0; 0]; % double(3), m, initial PCI position vector, 125 km altitude
in.s.traj.v_pci_ini = v0*[sind(fpa0); cosd(fpa0); 0]; % double(3), m/s, initial PCI inertial velocity vector

in.s.data_rate = 1;

%% Vehicle data

beta1 = 10;
beta2 = 100;

% Aerodynamics
in.v.aero.mode = uint8(2); % uint8, nd, aerodynamics mode
in.v.aero.aoa = 0; % double, rad, angle-of-attack
in.v.aero.cl = 0.0; % double, nd, lift coefficient
in.v.aero.cd = 1.53; % double, nd, drag coefficient
in.v.aero.area_ref = 200; % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1; % double, m, nose radius
temp = load(['.' filesep 'data' filesep 'aero_sphere_cone_60deg.mat']);
in.v.aero.table = temp.table;

in.v.mp.m_ini = beta1*in.v.aero.cd*in.v.aero.area_ref; % double, kg, vehicle mass

% Guidance
in.v.gnc.g.p.rate = 1; % nd, guidance rate

in.v.gnc.g.p.bank_mode = uint8(1); %  nd, constant bank
in.v.gnc.g.p.aoa_mode = uint8(1); %  nd, constant angle-of-attack
in.v.gnc.g.p.prop_mode = uint8(4); %  nd, SEJ-A guidance

% in.v.gnc.g.p_sej_a.target_flag = true; % nd
in.v.gnc.g.p_sej_a.iter_max = uint8(5); % nd
in.v.gnc.g.p_sej_a.A_sens_atm = 0.5; % m/s^2
in.v.gnc.g.p_sej_a.tgt_ap = 430000e3; % m
in.v.gnc.g.p_sej_a.tol_ap = 70000e3; % m
in.v.gnc.g.p_sej_a.alt_min = 0; % m
in.v.gnc.g.p_sej_a.alt_max = 500e3; % m
in.v.gnc.g.p_sej_a.K_dens_min = 0.1; % nd
in.v.gnc.g.p_sej_a.K_dens_max = 2.0; % nd
in.v.gnc.g.p_sej_a.K_dens_gain = 0.1; % nd
in.v.gnc.g.p_sej_a.t_max = 1500; % s
in.v.gnc.g.p_sej_a.tj_ini = [100; 101]; % s
in.v.gnc.g.p_sej_a.t_inc_max = 2.0; % s
in.v.gnc.g.p_sej_a.t_inc_min = 0.05; % s
in.v.gnc.g.p_sej_a.area_ref = [in.v.aero.area_ref; in.v.mp.m_ini/(in.v.aero.cd*beta2)]; % m^2
in.v.gnc.g.p_sej_a.cd = in.v.aero.cd; % nd
in.v.gnc.g.p_sej_a.cl = in.v.aero.cl; % nd
in.v.gnc.g.p_sej_a.mass = in.v.mp.m_ini; % kg
in.v.gnc.g.p_sej_a.p_r = in.p.r_e; % m
in.v.gnc.g.p_sej_a.mu = in.p.mu; % m^3/s^2
in.v.gnc.g.p_sej_a.j2 = in.p.j2; % nd
in.v.gnc.g.p_sej_a.npole = [0;0;1]; % nd
in.v.gnc.g.p_sej_a.omega = in.p.omega; % rad/s
in.v.gnc.g.p_sej_a.atm_table = in.p.atm.table(:,1:2);

in.v.gnc.g.p_sej_a.trust_region_ini = 10;

% Control
in.v.gnc.c.p.rate = in.s.traj.rate;

end % aa_ff_tcm_fullsim_in()
