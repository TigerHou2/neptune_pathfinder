% Stardust_in.m 
%   Create input structure for Stardust SRC Earth reentry
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
%
% Citations (all trajectory, aerodynamics, and decelerator data):
%   (1) Stardust Sample Return Capsule Design Experience: W.H. Willcockson
%   (2) Stardust Entry Reconstruction: P.N. Desai, G.D. Qualls
%   (3) Stardust Parachute Trajectory Performance Reconstruction:
%       A. Witkowski, M. Kandis
%   (4) Aerodynamics of Stardust Sample Return Capsule: R.A. Mitcheltree, 
%       R.G. Wilmoth, F.M. Cheatwood, G.j. Brauckman, F.A. Greene
%   (5) Stardust mission web page, NASA JPL, starust.jpl.nasa.gov
%
% Notes
%   * Data for multiple aerodyanmic models provided; must choose between
%   constant coefficients or table look-up
%   * Data provided for parachute, but asim currently does not have the
%   capability to trigger parachute deployment without a hypersonic
%   guidance algorithm

function in = stardust_in()
%#codegen

%% Define input structure
in = default_in;


%% Planetary data

in.p = get_planetary_data( 2, 1 ); % Load settings for Earth, exponential atmosphere model

%% Simulation data

% Termination conditions

% OR conditions
in.s.term.or.type(1) = int8(-1); % int8, nd, inequality - less than
in.s.term.or.var(1) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(1) = 0; % double, m, terminate at zero altitude

% Trajectory
right_asc = 205.7*pi/180; % right ascension, rad (5)
decl = 11.1*pi/180; % inclination, rad (5)
fpa0 = -8.2*pi/180;  % inertial FPA; rad (1)
az = 310.27*pi/180; % azimuth, rad (calculated based on right asc, decl, and landing site)
in.s.traj.rate = 10; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 140; % double, s, maximum allowable time
[in.s.traj.r_pci_ini, in.s.traj.v_pci_ini] = LLAVFA2RV_I(decl, right_asc, 135e3, fpa0, az, 12900, in.p.omega, 0, 0, in.p.r_e, in.p.r_p);

%% Vehicle data

in.v.mp.m_ini = 46; % double, kg, vehicle mass

% Aerodynamics
in.v.aero.mode = uint8(2); % uint8, nd, constant lift and drag coefficients
in.v.aero.aoa = 0; % double, rad, angle of attack (1)
in.v.aero.area_ref = 0.51887; % double, m^2, aerodynamic reference area (4)
in.v.aero.nose_radius = 0.2286; % double, m, nose radius (4)

% % Constant coefficients
% in.v.aero.cl = 0; % double, nd, lift coefficient (lift neglected)
% in.v.aero.cd = 1.368; % double, nd, drag coefficient (4) (Averaged @ 0 AOA)

% % Mach-dependent coefficients:
temp1 = load(['.' filesep 'data' filesep 'aero_sphere_cone_60deg.mat']); 
temp2 = [10.5 0 1.515;  %
         12.2 0 1.4992; %
         17.1 0 1.4939; %
         24.7 0 1.4816; % values found in (4)
         35.4 0 1.4828; %
         40.0 0 1.4959; %
         42.7 0 1.464]; %
temp3 = linspace(min(temp1.table(:,1)),max(temp1.table(:,1)),length(temp1.table(:,1))-length(temp2(:,1)))';
mach = [ temp3; temp2(:,1)];
cl = zeros(size(temp1.table(:,2)));
cd = [ interp1(temp1.table(:,1),temp1.table(:,3),temp3); temp2(:,3)];
in.v.aero.table = [mach,cl,cd];

% % Decelerator (no current capability in asim to deploy parachute without hypersonic guidance)
% in.v.decel.mode = uint8(1); % constant cd
% in.v.decel.cd = 1.1; % nd, drag coefficient (3)
% in.v.decel.d = 7.62; % m, main chute diameter (3)

%% GNC

% Guidance
in.v.gnc.g.p.rate = 10; % Hz
in.v.gnc.g.p.bank_mode = uint8(2); % nd, constant bank rate 
in.v.gnc.g.p.const_bank_rate = 1.4137; % rad/s, (2)
in.v.gnc.g.p_sej_e.v_chute = 0.16 * sqrt(in.p.atm.gamma*in.p.atm.R*in.p.atm.temp_ref); % m/s, nominal parachute deploy velocity, mach 0.16 (3)

% Control
in.v.gnc.c.p.decel_mode = uint8(1); % nd, decelerator deployment mode



end

