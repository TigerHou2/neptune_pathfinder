% traj_calcs.m 
%   Additional trajectory calculations
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
%   in - struct(1), md, input data structure
%   t - double(1), s, current time
%   y - double(n), misc, current state vector
%                        Currently: [x, y, z, u, v, w, m, downrange,
%                                          crossrange, heat load]'
%   veh - struct(1), multi, vehicle data structure
%       s.aoa - double(1), rad, angle-of-attack
%       s.bank - double(1), rad, bank angle
%       s.ssa - double(1), rad, sideslip angle
%       s.area_ref - double(1), m^2, current aerodynamic reference area
%       s.compute_trim - uint8(1), nd, trim angle-of-attack calculation flag
%   ctrl - struct(1), multi, control data structure
%   
% Output:
%   calcs - data structure, misc, state derivative vector

function calcs = traj_calcs( in, t, y, veh, ctrl )
%#codegen

%% Initialize
delta_mass = 0; % discrete mass change, kg
delta_V = [0;0;0]; % instantaneous delta-V, PCI inertial velocity, m/s

% Assign states
pos_ii = y(1:3); % Inertial position
vel_ii = y(4:6); % Inertial velocity
mass = y(7); % Mass
pos_ii_mag = norm(pos_ii); % Inertial position magnitude
vel_ii_mag = norm(vel_ii); % Inertial velocity magnitude
% Assign parameters
omega = in.p.omega;
gamma = in.p.atm.gamma;
rp    = in.p.r_p;
re    = in.p.r_e;
mu    = in.p.mu;
J2    = in.p.j2;


%% Transform the State

% Inertial to planet relative transformation
rot_angle = norm(omega)*t; % Rotation angle [rad]
[L_PI] = get_LPI(rot_angle);

% Inertial and planet centric velocities
pos_ii_hat = pos_ii/pos_ii_mag; % Inertial position vector direction
pos_pp = L_PI*pos_ii; % Position vector planet/planet [m]
pos_pp_mag = norm(pos_pp); % m, magnitude of position vector in PCPF
pos_pp_hat = pos_pp/pos_pp_mag; % nd, unit position vector in PCPF

V_pf_pci = vel_ii - cross(omega,pos_ii);
vel_pp = L_PI*(vel_ii - cross(omega,pos_ii)); % Velocity vector planet/planet [m/s]
vel_pp_hat = vel_pp/norm(vel_pp);
vel_pp_mag = norm(vel_pp);

% Angular Momentum Calculations
h_ii = cross(pos_ii,vel_ii); % Inertial specific angular momentum vector [m^2/s]
h_ii_mag = norm(h_ii); % Magnitude of inertial angular momenum vector [m^2/s]
h_pp = cross(pos_pp,vel_pp);
h_pp_mag = norm(h_pp);
h_pp_hat = h_pp/h_pp_mag;

% Inertial flight path angle
arg = median([-1, 1, h_ii_mag/(pos_ii_mag*vel_ii_mag)]); % limit to [-1,1]
gamma_ii = acos(arg);
if dot(pos_ii,vel_ii) < 0
    gamma_ii = -gamma_ii;
end

% Relative flight path angle
arg = median([-1, 1, h_pp_mag/(pos_pp_mag*vel_pp_mag)]); % limit to [-1,1]
gamma_pp = acos(arg);
if dot(pos_pp,vel_pp) < 0
    gamma_pp = -gamma_pp;
end


%% Derived Quantity Calculations

% Compute latitude and longitude
[lat,lon] = get_lat_lon(pos_pp,re,rp);
% Compute NED basis unit vectors
[uN,uE,uD] = get_unit_NED( lat, lon ); % nd

% compute azimuth
vN = dot(vel_pp,uN); % m/s
vE = dot(vel_pp,uE); % m/s
azi_pp = atan2(vE,vN); % rad

% Compute Altitude
if re == rp
    alt = pos_ii_mag - re;
else
    alt = get_alt(pos_pp,re,rp,lat);
end

% Get density, pressure, temperature, and winds
[rho,pres,T,wind] = get_atm_data(in,alt); 

% Convert wind to pp (PCPF) frame
wE = wind(1); % positive to the east, m/s
wN = wind(2); % positive to the north, m/s
wU = wind(3); % positive up, m/s
wind_pp = wN*uN + wE*uE - wU*uD; % wind velocity in pp frame, m/s
vel_pp_rw = vel_pp + wind_pp; % relative wind vector, m/s
vel_pp_rw_mag = norm(vel_pp_rw); % relative wind magnitude, m/s
vel_pp_rw_hat = vel_pp_rw / vel_pp_rw_mag; % relative wind unit vector, nd

% Dynamic pressure
q = 1/2*rho*vel_pp_rw_mag^2; % base on wind-relative velocity

% Mach number
a = sqrt(gamma*in.p.atm.R*T);
mach = vel_pp_mag/a;

% Compute c.g. location
if in.v.mp.mode == uint8(2)
    % vary c.g. position linearly with dynamic pressure
    if t < in.v.mp.t1
        cg = in.v.mp.cg;
    elseif t > in.v.mp.t2
        cg = in.v.mp.cg+in.v.mp.delta_cg;
    else
        m = (in.v.mp.delta_cg) / (in.v.mp.t2-in.v.mp.t1);
        cg = in.v.mp.cg + m*(t-in.v.mp.t1);
    end
else
    % constant c.g.
    cg = in.v.mp.cg;
end


%% Gravity Force Calculation
gravity_ii_mag_spherical = mu*mass/pos_ii_mag^2;
J2_pp = [-mu/pos_pp_mag^3*pos_pp(1)*-3/2*J2*(re/pos_pp_mag)^2*(5*pos_pp(3)^2/pos_pp_mag^2-1); ...
         0; ...
         -mu*pos_pp(3)/pos_pp_mag^3*-3/2*J2*(re/pos_pp_mag)^2*(5*pos_pp(3)^2/pos_pp_mag^2-3)] * mass;
J2_ii = L_PI'*J2_pp;
gravity_ii = gravity_ii_mag_spherical*(-pos_ii_hat) + J2_ii; % Inertial gravity force [N]


%% Vectors and Rotation Tensors of Interest
n1 = skew(-h_pp_hat);
R1 = eye(3) + sind(90)*n1 + (1-cosd(90))*n1*n1;
n2 = skew(vel_pp_rw_hat);
R2 = eye(3) + sin(veh.s.bank)*n2 + (1-cos(veh.s.bank))*n2*n2;


%% Vehicle Aerodynamic Forces
% Aerodynamics data
[cl,cd,veh] = get_aero_data(mach, veh, alt, vel_pp_rw_mag, q, cg, in); % Drag coefficient
% Force calculations
drag_pp_hat = -vel_pp_rw_hat; % Planet relative drag force direction
drag_pp = q*cd*veh.s.area_ref*drag_pp_hat; % Planet relative drag force vector
lift_pp_hat = R2*R1*vel_pp_rw_hat; % Planet relative lift force direction
lift_pp = q*cl*veh.s.area_ref*lift_pp_hat; % Planet relative lift force vector
drag_ii = L_PI'*drag_pp; % Inertial drag force vector
lift_ii = L_PI'*lift_pp; % Inertial lift force vector


%% Propulsion
thrust_pp = [0;0;0]; % N
thrust_pp_mag = 0; % N
thrust_ii = [0;0;0]; % N

% Constant thrust along planet-fixed velocity vector below given Mach
if ctrl.s.ignition
    % Calculate thrust
    thrust_pp_mag = in.v.prop.main.thrust_max; % N
    thrust_pp = -thrust_pp_mag * vel_pp_hat; % N
    thrust_ii = L_PI'*thrust_pp; % N, inertial thrust vector
    
    % Null aerodynamics
    drag_pp_hat = -vel_pp_hat; % Planet relative drag force direction
    drag_pp = 0*drag_pp_hat; % Planet relative drag force vector
    lift_pp_hat = R2*R1*vel_pp_hat; % Planet relative lift force direction
    lift_pp = 0*lift_pp_hat; % Planet relative lift force vector
    drag_ii = L_PI'*drag_pp; % Inertial drag force vector
    lift_ii = L_PI'*lift_pp; % Inertial lift force vector    
end


%% Decelerator Forces
if ctrl.s.deploy_decel == true
    switch in.v.decel.mode

        case 1 % Constant drag coefficient
            cd_decel = in.v.decel.cd; % nd
            decel_area = pi*(in.v.decel.d/2)^2; % m^2
            decel_drag_pp_mag = q*decel_area*cd_decel;
            decel_drag_pp = decel_drag_pp_mag*(-vel_pp_rw_hat);
            decel_drag_ii = L_PI'*decel_drag_pp;
            decel_lift_ii = [0;0;0];
            
        case 2 % Mach-dependent drag coefficient
            cd_decel = lin_interp(in.v.decel.table(:,1), in.v.decel.table(:,2), mach); % nd
            decel_area = pi*(in.v.decel.d/2)^2; % m^2
            decel_drag_pp_mag = q*decel_area*cd_decel;
            decel_drag_pp = decel_drag_pp_mag*(-vel_pp_rw_hat);
            decel_drag_ii = L_PI'*decel_drag_pp;
            decel_lift_ii = [0;0;0];
            
        case 3 % Mach-dependent drag coefficient with dispersions
            cd_decel_dat = lin_interp(in.v.decel.table(:,1), in.v.decel.table(:,2:4), mach); % nd
            if in.v.decel.K >= 0
                cd_decel = cd_decel_dat(2) + in.v.decel.K*(cd_decel_dat(3)-cd_decel_dat(2));
            else
                cd_decel = cd_decel_dat(2) - in.v.decel.K*(cd_decel_dat(1)-cd_decel_dat(2));
            end
            decel_area = pi*(in.v.decel.d/2)^2; % m^2
            decel_drag_pp_mag = q*decel_area*cd_decel;
            decel_drag_pp = decel_drag_pp_mag*(-vel_pp_rw_hat);
            decel_drag_ii = L_PI'*decel_drag_pp;
            decel_lift_ii = [0;0;0];            
            
        otherwise % turn off
            cd_decel = 0;
            decel_drag_ii = [0;0;0];
            decel_lift_ii = [0;0;0];
    end
else
    cd_decel = 0;
    decel_drag_ii = [0;0;0];
    decel_lift_ii = [0;0;0];
end


%% Total Force
% Total inertial external force vector on body [N]
force_ii = drag_ii + lift_ii + decel_drag_ii + decel_lift_ii + ...
    gravity_ii + thrust_ii;


%% Additional Calculations
% G-loading
g_loading = (norm(force_ii-gravity_ii)/mass)/in.c.earth_g;
% Heat rate
rn = in.v.aero.nose_radius;
heat_rate = in.p.k*sqrt(rho)/sqrt(rn)*vel_pp_mag^3;


%% Store calculated data

% Position and range
calcs.pos_ii = pos_ii; % Inertial position
calcs.pos_ii_mag = pos_ii_mag; % Inertial position magnitude
calcs.pos_pp = pos_pp; % Planet-relative position
calcs.pos_pp_mag = pos_pp_mag; % Planet-relative position magnitude - could be removed
calcs.lat = lat; % latitude
calcs.lon = lon; % longitude
calcs.alt = alt; % Altitude

% Velocity
calcs.vel_ii = vel_ii; % Inertial velocity, PCI frame
calcs.vel_ii_mag = vel_ii_mag; % Inertial velocity magnitude
calcs.gamma_ii = gamma_ii; % Inertial flight-path angle
calcs.vel_pp = vel_pp; % Planet-relative velocity
calcs.vel_pp_mag = vel_pp_mag; % Planet-relative velocity magnitude
calcs.gamma_pp = gamma_pp; % Planet-relative flight-path angle
calcs.azi_pp = azi_pp; % planet-relative azimuth angle
calcs.alt_rate = vel_pp_mag*sin(gamma_pp); % planet-relative altitude rate
calcs.ground_speed = vel_pp_mag*cos(gamma_pp); % ground speed
calcs.V_pf_pci = V_pf_pci; % Planet-relative velocity in PCI frame
calcs.v_wind_mag = vel_pp_rw_mag; % wind-relative velocity magnitude
calcs.delta_V = delta_V; % Instantaneous inertial delta-V, PCI frame

% Forces
calcs.force_ii = force_ii; % Total inertial external force vector
calcs.thrust_ii = thrust_ii; % Total inertial thrust vector
calcs.thrust_pp = thrust_pp; % Total planet relative thrust vector
calcs.thrust_pp_mag = thrust_pp_mag; % Total planet relative thrust vector magnitude - could be removed
calcs.gravity_ii = gravity_ii; % gravity vector
calcs.aero_ii = drag_ii + lift_ii; % aerodynamic force vector
calcs.drag_ii = drag_ii; % drag vector
calcs.lift_ii = lift_ii; % lift vector
calcs.drag_ii_mag = norm(drag_ii); % drag force magnitude, could be removed
calcs.lift_ii_mag = norm(lift_ii); % lift force magnitude, could be removed
calcs.decel_drag_ii = decel_drag_ii; % decelerator drag force vector, N
calcs.g_loading = g_loading; % G-loading, nd

% Aerodynamics
calcs.cl = cl; % lift coefficient, nd
calcs.cd = cd; % drag coefficient, nd
calcs.cd_decel = cd_decel; % decelerator drag coefficient, nd
calcs.LoD = cl/cd; % lift-to-drag ratio, could be removed
calcs.q = q; % dynamic pressure
calcs.mach = mach; % mach, nd
calcs.aoa = veh.s.aoa; % angle-of-attack, rad
calcs.bank = veh.s.bank; % bank, rad
calcs.ssa = veh.s.ssa; % sideslip angle, rad
calcs.area_ref = veh.s.area_ref; % aerodynamic reference area, m^2;

% Mass properties
calcs.mass = mass; % vehicle mass
calcs.cg = cg; % c.g. position
calcs.delta_mass = delta_mass; % change in mass

% Atmospheric parameters
calcs.rho = rho; % Density
calcs.T = T; % temperature
calcs.pres = pres; % pressure
calcs.wind = wind_pp; % planet-relative wind vector

% Heating
calcs.heat_rate = heat_rate; % stagnation-point convective heat rate


end % traj_calcs()
