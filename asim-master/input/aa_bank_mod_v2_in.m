%% aa_bank_mod_v2_in
%   Input file for bank angle modulation entry width corridor simulation.
% 
% Author:   Tiger Hou
% Created:  02/19/2020
% Modified: 02/27/2020
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
function in = aa_bank_mod_v2_in()
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

% Trajectory targets
LoD = 0.25; % nd
v0 = 30e3; % m/s
fpa0 = deg2rad(-7.8); % rad
alt0 = 500e3; % m
tgt_ap = 430000e3;
tgt_pe = 2500e3 + in.p.r_e;

in.s.traj.rate = 100; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 2500; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [(in.p.r_e + alt0); 0; 0]; % double(3), m, initial PCI position vector
in.s.traj.v_pci_ini = v0 .* [sin(fpa0); cos(fpa0); 0];  % Inertial velocity

in.s.data_rate = 1;

%% Vehicle data

in.v.mp.m_ini = 3300; % double, kg, vehicle mass

% Navigation
in.v.gnc.n.p.mode = uint8(1); % perfect navigation mode

% Aerodynamics
in.v.aero.mode = uint8(1); % uint8, nd, constant aerodynamic coefficients
in.v.aero.cl = 0.4;
in.v.aero.cd = in.v.aero.cl / LoD;
in.v.aero.area_ref = pi*(4.5/2)^2; % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1; % double, m, nose radius
temp = load(['data' filesep 'aero_MSL.mat']);

% GNC
in.v.gnc.g.p.bank_mode = uint8(5); % uint8, nd, analytical bank angle guidance

% Guidance parameters

in.v.gnc.g.p_bank_a.mass = in.v.mp.m_ini;
in.v.gnc.g.p_bank_a.area_ref = in.v.aero.area_ref;
in.v.gnc.g.p_bank_a.cl = in.v.aero.cl;
in.v.gnc.g.p_bank_a.cd = in.v.aero.cd;
in.v.gnc.g.p_bank_a.fpa_e = fpa0; % entry flight path angle
in.v.gnc.g.p_bank_a.fpa_f = -in.v.gnc.g.p_bank_a.fpa_e-deg2rad(1);
in.v.gnc.g.p_bank_a.beta = 1/in.p.atm.scale_height;% m^-1, inverse of atmosphere scale height
in.v.gnc.g.p_bank_a.R = norm(in.s.traj.r_pci_ini); % m, radius of atmospheric entry
in.v.gnc.g.p_bank_a.Ve = norm(in.s.traj.v_pci_ini); % m/s, velocity at atmospheric entry
in.v.gnc.g.p_bank_a.delta = in.p.mu / ...
    in.v.gnc.g.p_bank_a.R / in.v.gnc.g.p_bank_a.Ve^2; % nd, non-dimensional entry speed parameter [1] eqn.14
in.v.gnc.g.p_bank_a.rho_e = interp1(in.p.atm.table(:,1),in.p.atm.table(:,2),alt0);
in.v.gnc.g.p_bank_a.eps = sqrt(in.v.gnc.g.p_bank_a.R/in.v.gnc.g.p_bank_a.beta)*...
    in.v.gnc.g.p_bank_a.rho_e*in.v.aero.area_ref*in.v.aero.cd/in.v.mp.m_ini; % nd, non-dimensional entry altitude parameter [1] eqn.13

in.v.gnc.g.p_bank_a.tgt_ap = tgt_ap;
in.v.gnc.g.p_bank_a.tgt_pe = tgt_pe;

% we are fudging the sma for now
sma = (in.p.r_e - 350e3 + in.v.gnc.g.p_bank_a.tgt_ap) / 2;
vf = sqrt(in.p.mu*(2/in.v.gnc.g.p_bank_a.R - 1/sma));
in.v.gnc.g.p_bank_a.x_f = 2*log(in.v.gnc.g.p_bank_a.Ve/vf);

% this basically always gives y_f = 1
% syms y positive
% eqn = 0 == in.v.gnc.g.p_bank_a.eps*LoD/2*(y-1) + ...
%     (1-in.v.gnc.g.p_bank_a.delta)/sqrt(in.v.gnc.g.p_bank_a.beta*in.v.gnc.g.p_bank_a.R)*log(y);
% soln = solve(eqn,y);
% in.v.gnc.g.p_bank_a.y_f = double(max(soln));
in.v.gnc.g.p_bank_a.y_f = 1;

% convert variables to km for guidance code implementation
in.v.gnc.g.p_bank_a.beta = in.v.gnc.g.p_bank_a.beta * 1000; % km^-1
in.v.gnc.g.p_bank_a.R = in.v.gnc.g.p_bank_a.R / 1000;
in.v.gnc.g.p_bank_a.Ve = in.v.gnc.g.p_bank_a.Ve / 1000;

% in.v.gnc.g.p_bank_a.omega = in.p.omega; % rad/s, planet angular velocity vector


end % aa_bank_mod_v2_in()
