% aa_define_in.m 
%   Define input structure.
%
% Author:   Tiger Hou
% Created:  02/09/2020
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
%   in - data structure, misc, empty input data structure

function [in] = aa_define_in()
%#codegen


%% Constants data structure

c1 = struct( ...
    'Pa2psf', double(0.0208854342), ... % Convert Pascals to pounds per square foot
    'm2nmi', double(0.000539956803), ... % Convet meters to nautical miles
    'm2mi', double(0.000621371192), ... % Convert meters to miles
    'm2ft', double(3.2808399), ... % Convert meters to feet
    'm2in', double(39.3700787), ... % Convert meters to inches
    'r2d', double(57.2957795), ... % Convert radians to degrees
    'N2lbf', double(0.224808943), ... % Convert Newtons to pounds force
    'kg2lbm', double(2.20462262), ... % Convert Newtons to pounds mass
    'J2BTU', double(0.00094781712), ... % Convert joules to BTU
    'K2degR', double(9/5), ... % Convert degrees Kelvin to degrees Rankine
    'earth_g', double(9.81) ); % Earth's surface gravity, m/s^2


%% Simulation data structure

% Construct trajectory data structure
and_or = struct( ...
    'type', int8(zeros(5,1)), ...
    'var', uint8(zeros(5,1)), ...
    'value', double(zeros(5,1)), ...
    'direction', int8(zeros(5,1)) );

term = struct( ...
    'and', and_or, ...
    'or', and_or );

traj = struct( ...
    'rate', double(0), ... % Integration rate, Hz
    't_ini', double(0), ... % Initial time, s
    't_max', double(0), ... % Maximum time, s
    'aoa_ini', double(0), ... % Initial angle of attack, rad
    'bank_ini', double(0), ... % Initial bank angle, rad
    'ssa_ini', double(0), ... % Initial side slip angle, rad
    'lat', double(0), ... % Latiitude, deg
    'lon', double(0), ... % Longitude, deg
    'alt', double(0), ... % Altitude, m
    'gamma_pp', double(0), ... % Flight path angle, rotating planetary frame, rad
    'az', double(0), ... % Azimuth, deg
    'dr_ini', double(0), ... % Initial downrange, m
    'cr_ini', double(0), ... % Initial crossrange, m
    'vel_pp_mag', double(0), ... % Velocity magnitude, rotating planetary frame, m/s
    'r_pci_ini', zeros(3,1), ... % Initial position vector, inertial frame, m
    'v_pci_ini', zeros(3,1) ... % Initial velocity vector, inertial frame, m/s
    );
    
% Construct simulation data structure
s = struct( ...
    'data_rate', double(1), ... % Simulation time step
    'term', term, ...
    'traj', traj );


%% Planetary data structure

% Construct atmosphere data structure
atm = struct( ...
    'mode', uint8(0), ...
    'R', double(0), ... % Ideal gas constant, J/(kg*K)
    'gamma', double(0), ... Ratio of specific heats, nd
    'scale_height', double(0), ... % Atmospheric scale height, m
    'dens_ref', double(0), ... % Surface density, kg/m^3
    'pres_ref', double(0), ... % Surface pressure, Pa
    'temp_ref', double(0), ... % Surface temperature, K
    'table', zeros(1000,7) ); % Altitude dependent values

% Construct planetary data structure
p = struct( ...
    'planet', uint8(0), ...
    'r_e', double(0), ... % Equatorial radius of planet, m
    'r_p', double(0), ... % Polar radius of planet, m
    'r_m', double(0), ... % Volumetric mean radius of planet, m
    'mass', double(0), ... % Mass of planet, kg   
    'mu', double(0), ... % Gravitation paramaeter of planet, m^3/s^2
    'surface_g', double(0), ... % Gravitational surface acceleration, m/s^2
    'j2', double(0), ... % Oblateness of planet, nd
    'k', double(0), ... % Sutton-Graves heating coefficient, kg^0.5/m 
    'omega', zeros(3,1), ... % Angular velocity vector of planet, inertial frame, rad
    'atm', atm ); % Atmosphere structure


%% Vehicle data structure


% Construct aerodynamics data structure
aero = struct( ...
    'mode', uint8(0), ...
    'r_cg', zeros(3,1), ... % c.g. position vector, structural frame, m
    'r_cp', zeros(3,1), ... % c.p. position vector, structural frame, m
    'aoa', double(0), ... % constant angle-of-attack, rad
    'cl', double(0), ... % constant lift coefficient, nd
    'cd', double(0), ... % constant drag coefficient, nd
    'area_ref', double(0), ... % aerodynamic refernece area, m^2
    'chord_ref', double(0), ... % aerodynamic reference chord, m
    'span_ref', double(0), ... % aerodynamic reference span, m
    'table', zeros(100,3), ... % mach-dependent cl and cd, nd
    'nose_radius', double(0) ); % effective nose radius, m

% Construct GNC data structure
guid = aa_define_guid; % Guidance structure
ctrl = define_ctrl; % Control structure
nav = define_nav; % Navigation structure
% guidance parameters
g = struct( ...
    'p', guid.p, ... 
    'p_msl', guid.msl.p, ...
    'p_sej_a', guid.sej_a.p, ...
    'p_dma_cv', guid.dma_cv.p, ...
    'p_sej_e', guid.sej_e.p, ...
    'p_gt', guid.gt.p, ...
    'p_bank_a', guid.bank_a.p, ...
    'p_pathfinder', guid.pathfinder.p ...
    );
% navigation parameters
n = struct( ...
    'p', nav.p ...
    );
% control parameters
c2 = struct( ...
    'p', ctrl.p ...
    );
% GNC parameters
gnc = struct( ...
    'g', g, ...
    'n', n, ...
    'c', c2 ...
    );

% Construct propulsion data structure
main = struct( ...
    'isp', double(1), ... % Main thrust specific impulse (must be non-zero), s
    'thrust_max', double(0) ); % Maximum main thrust, N
rcs = struct( ...
    'isp', double(1), ... % Reentry control specific impulse (must be non-zero), s
    'thrust_max', double(0) ); % Reentry control maximum thrust, N
prop = struct( ... 
    'main', main, ...
    'rcs', rcs );
  
% Construct decelerator data structure
decel = struct( ...
    'mode', uint8(0), ...
    'd', double(0), ... % Decelerator diameter, m
    'cd', double(0), ... % Coefficient of drag, nd
    'K', double(0), ... % Maskell bluff-body blockage factor?, nd
    'table', double(zeros(44,4)) );
  
% Construct mass properties
mp = struct( ...
    'mode', uint8(0), ... 
    'm_ini', double(0), ... % Initial mass, kg
    'm_jettison', double(0), ... % Jettisoned mass, kg
    'cg', zeros(3,1), ... % Center of gravity position vector, inertial frame, m
    'delta_cg', zeros(3,1), ... % Change in center of gravity, m/s
    't1', double(0), ... % Initial time, s
    't2', double(0) ); % Final time, s

% Construct vehicle data structure
v = struct( ...
    'aero', aero, ...
    'gnc', gnc, ...
    'prop', prop, ...
    'decel', decel, ...
    'mp', mp );


%% Construct combined input data structure

in = struct( ...
    'c', c1, ...    
    's', s, ...
    'p', p, ...
    'v', v);


end % aa_define_in()