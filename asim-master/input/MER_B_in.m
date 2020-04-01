% MER_B_in.m 
%   Create input structure for MER-B ballistic entry at Mars
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

function in = MER_B_in()
%#codegen

%% Define input structure

in = default_in;


%% Planetary data

in.p = get_planetary_data( 3, 1 ); % Load settings for Mars, table atmopshere model


%% Simulation data

% Termination conditions
in.s.term.or.type(1) = int8(0); % int8, nd, crossing/equality condition
in.s.term.or.var(1) = uint8(1); % uint8, nd, termination mode (altitude)
in.s.term.or.value(1) = -2000; % double, m, terminate at -1 km MOLA
in.s.term.or.direction(1) = int8(-1); % int8, nd, negative crossing

% Trajectory
v0 = 5500; % m/s
fpa0 = -11.47; % deg

in.s.traj.rate = 10; % double, Hz, integration rate
in.s.traj.t_ini = 0; % double, s, initial time
in.s.traj.t_max = 1000; % double, s, maximum allowable time
in.s.traj.r_pci_ini = [(in.p.r_e + 125e3); 0; 0]; % double(3), m, initial PCI position vector, 125 km altitude
in.s.traj.v_pci_ini = v0*[sind(fpa0); cosd(fpa0); 0]; % double(3), m/s, initial PCI inertial velocity vector


%% Vehicle data

in.v.mp.m_ini = 832; % double, kg, vehicle mass

% Aerodynamics
in.v.aero.mode = uint8(2); % uint8, nd, aerodynamics mode
in.v.aero.area_ref = in.v.mp.m_ini/(1.6*94); % double, m^2, aerodynamic reference area
in.v.aero.nose_radius = 1; % double, m, nose radius
temp = load(['.' filesep 'data' filesep 'aero_sphere_cone_70deg.mat']);
in.v.aero.table = temp.table;

% GNC
in.v.gnc.g.p.bank_mode = uint8(1); % uint8, nd, constant bank
in.v.gnc.g.p.const_bank = double(0);


end % MER_B_in()