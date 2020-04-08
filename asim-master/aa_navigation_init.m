% aa_navigation_init.m
%   Initialize navigation model
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
%   in - struct(1), multi, input data structure
%
% Output:
%   nav - struct(1), multi, navigation data structure

function [ nav, nav_rnd ] = aa_navigation_init( in )

%% Construct navigation data structure
nav = aa_define_nav;

%% Copy navigation parameters
nav.p = in.v.gnc.n.p;
if nav.p.mode ~= 2 && nav.p.mode ~= 3
    nav.p.omega = in.p.omega;
    nav.p.r_e = in.p.r_e;
    nav.p.r_p = in.p.r_p;
end

%% Initialize random numbers for navigation error model
t_length = round((in.s.traj.t_max-in.s.traj.t_ini)*in.s.traj.rate)+1;
if nav.p.mode == 2
    rng(nav.p.seed,'twister'); % use input seed
    nav_rnd = randn(9,t_length); % create repeatable set of random numbers on U[-1,1] for navigation model
    nav.s.x_ercv = (1 - exp(-2*nav.s.dt/nav.p.tau))*nav_rnd(:,1);
    nav.s.rva_error = nav.p.P_SS*nav.s.x_ercv;
elseif nav.p.mode == 3
    rng(nav.p.seed,'twister'); % use input seed
    nav_rnd = randn(9,t_length); % create repeatable set of random numbers on U[-1,1] for navigation model
    nav.s.x_ercv = (1 - exp(-2*nav.s.dt/nav.p.tau))*nav_rnd(:,1);
    nav.s.rva_error = nav.p.P_SS*nav.s.x_ercv;
    nav.s.scale = 1 + randn(3,1).*nav.p.scale/3; % create fixed random scale factor for r,v,a
    nav.s.bias  = randn(9,1).*nav.p.bias/3;  % create fixed bias for vectors r,v,a (divide by 3 because nav.p.bias given in 3-sigma)
    z = rand(3,1) .* (1-cos(nav.p.tilt)) + cos(nav.p.tilt);
    t = rand(3,1) .* 2*pi; % note that the uniform distribution is used here
    tilt = [z,sqrt(1-z.^2).*cos(t), sqrt(1-z.^2).*sin(t)]; % create fixed axis tilt for r,v,a (1st row = r, 2nd row = v, 3rd row = a)
    x = [1;0;0];
    Rr = aa_getRotnMat(cross(x,tilt(1,:)'),acos(tilt(1,:)*x));
    Rv = aa_getRotnMat(cross(x,tilt(2,:)'),acos(tilt(2,:)*x));
    Ra = aa_getRotnMat(cross(x,tilt(3,:)'),acos(tilt(3,:)*x));
    nav.s.tilt_r = Rr(:);
    nav.s.tilt_v = Rv(:);
    nav.s.tilt_a = Ra(:);
else
    nav_rnd = zeros(9,t_length+1);
    nav.s.x_ercv = zeros(9,1);
    nav.s.rva_error = zeros(9,1);
end

%% Set states
nav.s.r_pci = in.s.traj.r_pci_ini + nav.s.rva_error(1:3);
nav.s.v_inrtl_pci = in.s.traj.v_pci_ini + nav.s.rva_error(4:6);
nav.s.a_sens_pci = zeros(3,1) + nav.s.rva_error(7:9);
% Set derived states to zero
nav.s.r_pcpf = zeros(3,1);
nav.s.v_pf_pci = zeros(3,1);
nav.s.v_pf_pcpf = zeros(3,1);
nav.s.a_sens_pcpf = zeros(3,1);
nav.s.alt = 0;
% Time and rate parameters
nav.s.t_ini = in.s.traj.t_ini;
nav.s.dt = 1/nav.p.rate; % seconds
nav.s.t = -nav.s.dt; % start one time step back
if nav.p.rate == 0
    nav.s.sn_ratio = 1; % execute every sim time step
else
    nav.s.sn_ratio = round(in.s.traj.rate/nav.p.rate); % sim/navigation rate ratio, nd
end

end % aa_naviation_init()
