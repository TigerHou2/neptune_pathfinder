% red_dragon_in.m 
%   Create input structure with default values for Red Dragon mission - 
%   EDL of the SpaceX Dragon capsule at Mars with gravity turn.
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
%   in - struct, multi, input structure with default values

function in = red_dragon_in()
%#codegen


%% State Inputs
alt0 = 125e3; % m
fpa0 = -15; % deg
v0 = 6000; % m/s


%% Define input structure
in = define_in;


%% Constants
in.c.earth_g = 9.81;


%% Planetary data
in.p = get_planetary_data( 3, 2 ); % Load settings for Mars, table look-up atmosphere


%% Simulation data

% Termination conditions
% OR conditions
ii = 1;
in.s.term.or.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.or.var(ii) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(ii) = -1e3; % double, m, terminate at -1 km MOLA
ii = ii + 1;

in.s.term.or.type(ii) = int8(1); % int8, nd, inequality - greater than
in.s.term.or.var(ii) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(ii) = 200e3; % double, m, terminate at 150 km altitude
ii = ii + 1;

in.s.term.or.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.or.var(ii) = uint8(2); % uint8, nd, termination mode (mass)
in.s.term.or.value(ii) = 0; % double, kg, terminate at zero mass
ii = ii + 1;

% AND conditions
ii = 1;
in.s.term.and.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.and.var(ii) = uint8(4); % uint8, nd, termination mode (vert. pf vel.)
in.s.term.and.value(ii) = 10; % double, m/s, terminate at 10 m/s velocity
ii = ii + 1;

in.s.term.and.type(ii) = int8(-1); % int8, nd, inequality - less than
in.s.term.and.var(ii) = uint8(3); % uint8, nd, termination mode (Mach)
in.s.term.and.value(ii) = 2; % double, nd, ok to terminate below Mach 2
ii = ii + 1;


% Trajectory
in.s.traj.rate = 10; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 1500; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [in.p.r_e + alt0; 0; 0]; % double(3), m, initial PCI position vector, 125 km altitude
in.s.traj.v_pci_ini = v0*[sind(fpa0); cosd(fpa0); 0]; % double(3), m/s, initial PCI inertial velocity vector
in.s.traj.bank_ini = 0; % double, rad, lift up

in.s.data_rate = 2;

%% Vehicle data

in.v.mp.m_ini = 7200; % double, kg, vehicle dry mass (4200 kg) plus prop (3000 kg)

% Aerodynamics
in.v.aero.mode = uint8(2); % uint8, nd, aerodynamics mode
in.v.aero.aoa = 0; % double, rad, angle-of-attack
in.v.aero.cl = 0.286; % double, nd, approx. lift coefficient
in.v.aero.cd = 1.567; % double, nd, approx. drag coefficient
in.v.aero.area_ref = pi*(3.6/2)^2; % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1.8; % double, m, nose radius

% scale Apollo aerodynamics to L/D of 0.18
temp = load(['.' filesep 'data' filesep 'aero_apollo.mat']);
temp.table(:,2) = temp.table(:,2)*0.72;
temp.table(:,3) = temp.table(:,3)*1.23;
in.v.aero.table = temp.table;

% figure; hold on; box on; grid on;
% plot(temp.table(:,1),temp.table(:,2),'b');
% plot(temp.table(:,1),temp.table(:,3),'r');
% plot(temp.table(:,1),temp.table(:,2)./temp.table(:,3),'g');

% Guidance
in.v.gnc.g.p.bank_mode = uint8(1); % nd, constant bank
in.v.gnc.g.p.aoa_mode = uint8(1); % nd, constant AOA
in.v.gnc.g.p.prop_mode = uint8(2); % nd, gravity turn start logic
in.v.gnc.g.p.rate = 1; % Hz, guidance rate

in.v.gnc.g.p_gt.tgt_V_mag = 0; % target velocity, m/s
in.v.gnc.g.p_gt.t_max = 500; % s
in.v.gnc.g.p_gt.tgt_alt = -0.5e3; % target altitude, m
in.v.gnc.g.p_gt.thrust = 200e3; % N, available thrust
in.v.gnc.g.p_gt.m_ini = in.v.mp.m_ini; % kg
in.v.gnc.g.p_gt.r_e = in.p.r_e; % m
in.v.gnc.g.p_gt.g = in.p.surface_g; % m/s^2
in.v.gnc.g.p_gt.g0 = in.c.earth_g; % m/s^2
in.v.gnc.g.p_gt.isp = 240; % s
in.v.gnc.g.p_gt.h = 1; % s
in.v.gnc.g.p_gt.V_mag_min = 3000; % m/s

% Control
in.v.gnc.c.p.rate = in.s.traj.rate;

% Propulsion
in.v.prop.main.isp = in.v.gnc.g.p_gt.isp; % double, s, specific impulse - hydrazine mono
in.v.prop.main.thrust_max = in.v.gnc.g.p_gt.thrust; % double, N, maximum thrust


end % red_dragon_gt_in()
