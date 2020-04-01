%% aa_bank_mod_v1
%
% Author:   Tiger Hou
% Created:  02/09/2020
% Modified: 02/18/2020
%
% DESCRIPTION
%   - This script simulates aerocapture at Neptune using bank angle control
%       Variables include L/D, V0, EFPA, and Ap Tolerance
%   - For each configuration, two trajectories are flown (fully lift-down
%       and fully lift-up) to find the maximum boundaries.
%
% SOURCES:
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies
%   - [3] Neptune Aerocapture Systems Analysis
%

%% Initialization

close all hidden
clear;clc
setup
in = aa_bank_mod_v1_in();    % load the input file

% create Neptune atmosphere dispersion model
% (currently we are not using it)
% nep_atm = aa_make_nep_atm(in);

disp('Initialization complete!')
disp('')


%% Data Saving Options

% get filename for data file
opts.Interpreter = 'tex';
fName = inputdlg( '\fontsize{12} Enter data file name', ...
                  'User Input', [1,50], ...
                 {'data_file'}, opts);
if isempty(fName)
    warning('No file name provided. Using default name "data_file.mat".');
    fName = 'data_file';
else
    fName = fName{1};
end

%% Run Simulation

disp('Starting sim...')

% orbiting body gravitational parameter
mu = in.p.mu;

% simulation ranges
ld_range   = [0.15,0.25,0.3,0.4];
v_init     = (25:0.5:35)*1e3;
gammas     = -12:0.2:-5.0;

% get size of each dimension
ld_len = length(ld_range);
vi_len = length(v_init);
ga_len = length(gammas);
datsize = ld_len*vi_len*ga_len;

disp(['Data size: ' num2str(datsize) ...
      ' | Estimated time: ' num2str(datsize/5) ' seconds'])
disp('')

in_cell = cell(ld_len,vi_len,ga_len);

for i = 1:ld_len
    for j = 1:vi_len
        for k = 1:ga_len
            in_cell{i,j,k} = [ld_range(i),v_init(j),gammas(k)];
        end
    end
end

result = NaN(datsize,5);

parfor i = 1:datsize
    % start simulation
    disp(['Progress: ' num2str(i)])
    temp = in_cell{i};
    LoD  = temp(1);
    v0   = temp(2);
    fpa0 = temp(3);
    
    this_in = in;
    
    this_in.s.traj.v_pci_ini = v0 .* [sind(fpa0); cosd(fpa0); 0];  % Inertial velocity
    this_in.v.gnc.g.p.const_bank = pi; % bank angle, 0 = lift up, pi = lift down [rad]
    this_in.v.gnc.g.p_sej_a.tgt_ap = 430000e3; % target apoapsis [m]
    this_in.v.aero.cd = this_in.v.aero.cl / LoD; % (fixed) coefficient of drag [nd]
    
    % ====================== lift up trajectory =========================
    this_in.v.gnc.g.p.const_bank = 0; % bank angle [rad]
    dat_temp = aa_main_mex(this_in);
    
    % calculate apoapsis of orbit
    idx = sum(~isnan(dat_temp.traj.time)); % termination index
    r = dat_temp.traj.pos_ii(idx,:)';      % termination position
    v = dat_temp.traj.vel_ii(idx,:)';      % termination velocity
    ap_max = norm(r)/(2 - norm(r)*dot(v,v)/mu);
    
    % ====================== lift down trajectory =========================
    this_in.v.gnc.g.p.const_bank = pi; % bank angle [rad]
    dat_temp = aa_main_mex(this_in);
    
    % calculate apoapsis of orbit
    idx = sum(~isnan(dat_temp.traj.time)); % termination index
    r = dat_temp.traj.pos_ii(idx,:)';      % termination position
    v = dat_temp.traj.vel_ii(idx,:)';      % termination velocity
    ap_min = norm(r)/(2 - norm(r)*dot(v,v)/mu);
    
    % save to results
    result(i,:) = [LoD, v0, fpa0, ap_max, ap_min];
end

save(['aa-contours/' fName '.mat'], ...
    'result', ...
    'ld_range', 'v_init', 'gammas', ...
    'ld_len', 'vi_len', 'ga_len');

disp('Simulation complete!')
disp('')

%% Plot Acceptable Configurations
% In the simulation, we varied L/D, V0, and EFPA, and for each
%   configuration we simulated a fully lift-down and a fully lift-up
%   trajectory. This establishes the range of reachable orbits using bank
%   angle control for any specific (L/D, V0, EFPA).
%
% Now, we define a range of target orbits. Then, 
%   - If the lift-up trajectory (which has the higher Ap) has an Ap that is
%       lower than the minimum acceptable Ap, reject.
%   - If the lift-down trajectory (which has the lower Ap) has an Ap that is
%       higher than the maximum acceptable Ap, reject.
%   - We are now left with configurations of (L/D, V0, EFPA) for which an
%       acceptable orbit can be achieved between lift-up and lift-down
%       a.k.a within the control range.

ap_tgt = 430000e3;
ap_tol = 320000e3;

out = result;

out(out(:,4)<(ap_tgt-ap_tol),:) = []; % if max ap smaller than min acceptable ap, reject
out(out(:,5)>(ap_tgt+ap_tol),:) = []; % if min ap larger than max acceptable ap, reject

out = rmoutliers(out);

temp = cell(ld_len,1);

% sort by L/D
for i = 1:ld_len
    tmp = out;
    tmp(tmp(:,1)~=ld_range(i),:) = [];
    temp{i} = tmp;
end

figure
hold on
lgd = {};
for i = 1:ld_len
    tmp = temp{i};
%     scatter3(tmp(:,2)/1e3,tmp(:,3),tmp(:,4)/1e3)  % max ap
%     scatter3(tmp(:,2)/1e3,tmp(:,3),tmp(:,5)/1e3)  % min ap
    scatter3(tmp(:,2)/1e3,tmp(:,3),(tmp(:,4)-tmp(:,5))/1e3)  % ap diff
    lgd{end+1} = strcat('L/D = ',num2str(ld_range(i)));
end
hold off

grid on
title('Entry Theoretial Corridor Width, Z-Axis = Achieved Ap')
xlabel('Initial Velocity, km/s')
ylabel('Flight Path Angle, deg')
zlabel('Lift-Up/Lift-Down Apoapsis Difference, km')
legend(lgd)


%% Single Test Run

close all hidden
v0 = 29e3;
fpa0 = -5.9; % -7.3 for bank = 0, LoD = 0.28  ||  
LoD = 0.20;
in.s.traj.v_pci_ini = v0 .* [sind(fpa0); cosd(fpa0); 0];  % Inertial velocity
in.v.gnc.g.p.const_bank = pi; % bank angle, 0 = lift up, pi = lift down [rad]
in.v.gnc.g.p_sej_a.tgt_ap = 430000e3; % target apoapsis [m]
in.v.aero.cd = in.v.aero.cl / LoD; % (fixed) coefficient of drag [nd]
test = aa_main_mex(in);

idx = sum(~isnan(test.traj.time)); % find index of termination point

r = test.traj.pos_ii(idx,:)'; % position vector [m]
v = test.traj.vel_ii(idx,:)'; % velocity vector [m/s]

res = 100;  % calculate 100 orbit points for hyperbolic orbits [nd]
start_day = 0.0001; % set the middle point of the orbit to be periapsis [days]
mu = in.p.mu; % get gravitational parameter of orbiting body [m^3/s^2]

ah = norm(r)/(2 - norm(r)*dot(v,v)/mu); % calculate semimajor axis length [m]
disp(ah)


%% Plot Orbit for Test Run

if ah >= 0
    dur = 2*pi*sqrt(ah^3/mu)/3600/24;  % orbit propagation duration [days]
else
    dur = 0.1;  % hyperbolic. plot 0.1 days centered around periapsis
%     ap = in.v.gnc.g.p_sej_a.tgt_ap;
%     dur = 2*pi*sqrt(ap^3/mu)/3600/24;
end

pos = Get_Orb_Points(r,v,mu,res,dur,start_day);
e_vec = ((dot(v,v)-mu/norm(r))*r - dot(r,v)*v)/mu;

hold on

[x,y,z] = sphere;
x = x*in.p.r_m;
y = y*in.p.r_m;
z = z*in.p.r_m;
surf(x,y,z)
plot3(pos(:,1), pos(:,2), pos(:,3),'LineWidth',2.5,'Color','Red')

% view(cross(r,v))
view([1,1,1])
pbaspect([1 1 1])
axis equal
grid on
hold off