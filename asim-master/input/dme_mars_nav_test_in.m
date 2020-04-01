% dme_mars_in.m 
%   Create input structure for drag-modulated entry at Mars. Vehicle based
%   on idea of landing an MER-class payload with MSL-class accuracy,
%   subject to current launch vehicle maximum diameter constraints
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

function in = dme_mars_nav_test_in()
%#codegen

%% Define input structure

in = default_in;


%% Planetary data

in.p = get_planetary_data( 3, 3 ); % Load settings for Mars, exponential atmosphere


%% Simulation data

% % Termination conditions
in.s.term.or.type(1) = int8(0); % int8, nd, crossing/equality condition
in.s.term.or.var(1) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(1) = -2000; % double, m, terminate at -2 km MOLA
in.s.term.or.direction(1) = int8(-1); % int8, nd, negative crossing

% in.s.term.or.type(1) = int8(0); % int8, nd, crossing/equality condition
% in.s.term.or.var(1) = uint8(4); % uint8, nd, termination mode (altitude)
% in.s.term.or.value(1) = 460; % double, m, terminate at -2 km MOLA
% in.s.term.or.direction(1) = int8(-1); % int8, nd, negative crossing

% Trajectory (MER-B entry conditions)
% v0 = 5500; % m/s
% fpa0 = -11.47; % deg

in.s.traj.rate = 10; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 2000; % double, s, maximum allowable time

% state
in.s.traj.alt = 125e3; % initial altitude, m
in.s.traj.lat = 0; % initial latitude, rad
in.s.traj.lon = 0; % initial longitude, rad
in.s.traj.vel_pp_mag = 5500; % initial velocity, m/s
in.s.traj.gamma_pp = -11*pi/180; % initial flight-path, deg->rad
in.s.traj.az = 90*pi/180; % initial azimuth, deg->rad
[in.s.traj.r_pci_ini, in.s.traj.v_pci_ini] = LLAVFA2RV_I( ...
    in.s.traj.lat, in.s.traj.lon, in.s.traj.alt, ...
    in.s.traj.gamma_pp, in.s.traj.az, in.s.traj.vel_pp_mag, ...
    in.p.omega, 0, 0, in.p.r_e, in.p.r_p);


%% Vehicle data

in.v.mp.m_ini = 830; % double, kg, vehicle mass at entry

% Aerodynamics
in.v.aero.mode = uint8(2); % uint8, nd, aerodynamics mode (table look-up)
in.v.aero.area_ref = pi*(4.5/2)^2; % double, m^2, aerodynamic reference area, assumes 4.7 diameter possible
in.v.aero.nose_radius = 1; % double, m, nose radius
temp = load(['.' filesep 'data' filesep 'aero_sphere_cone_70deg.mat']);
in.v.aero.table = temp.table;

% GNC
in.v.gnc.g.p.bank_mode = uint8(1); % uint8, nd, constant bank
in.v.gnc.g.p.const_bank = double(0);


% Navigation
in.v.gnc.n.p.mode = uint8(2);
in.v.gnc.n.p.rate = in.s.traj.rate;
in.v.gnc.n.p.seed = uint32(1);
in.v.gnc.n.p.tau = 3600; % s
in.v.gnc.n.p.omega = in.p.omega;
in.v.gnc.n.p.r_e = in.p.r_e;
in.v.gnc.n.p.r_p = in.p.r_p;
% in.v.gnc.n.p.rva_error_ini = zeros(9,1);
P_SS = zeros(9);
P_SS(1:6,1:6) = [ ...
    2.981933182e-02  5.369876516e-02  7.337381389e-02  3.062525841e-05  1.173553844e-05  9.742939383e-06; ...
    5.369876516e-02  1.144226269e-01  1.498759926e-01  6.301336474e-05  2.198443978e-05  1.900429607e-05; ...
    7.337381389e-02  1.498759926e-01  1.983781389e-01  8.324339215e-05  2.974134394e-05  2.542989381e-05; ...
    3.062525841e-05  6.301336474e-05  8.324339215e-05  3.494835789e-08  1.243167048e-08  1.065436605e-08; ...
    1.173553844e-05  2.198443978e-05  2.974134394e-05  1.243167048e-08  4.663582047e-09  3.905447999e-09; ...
    9.742939383e-06  1.900429607e-05  2.542989381e-05  1.065436605e-08  3.905447999e-09  3.309286454e-09 ];
in.v.gnc.n.p.P_SS = P_SS*(1000^2);

end % dme_mars_in()