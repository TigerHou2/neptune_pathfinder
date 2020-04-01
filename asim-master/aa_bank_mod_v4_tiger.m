%% aa_bank_mod_v4_tiger
% aa_pathfinder
%
% Author:   Tiger Hou
% Created:  02/29/2020
% Modified: 03/03/2020
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

%% Single Test Run

close all hidden
clear;clc
setup
in = aa_pathfinder_in();    % load the input file

disp('Initialization complete!')
disp('')

test = aa_main_mex(in);
% test = aa_main(in);

idx = sum(~isnan(test.traj.time)); % find index of termination point

r = test.traj.pos_ii(idx,:)'; % position vector [m]
v = test.traj.vel_ii(idx,:)'; % velocity vector [m/s]

res = 100;  % calculate 100 orbit points for hyperbolic orbits [nd]
start_day = 0.0001; % set the middle point of the orbit to be periapsis [days]
mu = in.p.mu; % get gravitational parameter of orbiting body [m^3/s^2]

[a,e] = Get_Orb_Params(r,v,mu);
disp(a)

ap = a * (1 + norm(e));
pe = a * (1 - norm(e));


%% Plot Orbit for Test Run

if a >= 0
    dur = 2*pi*sqrt(a^3/mu)/3600/24;  % orbit propagation duration [days]
else
    dur = 0.1;  % hyperbolic. plot 0.1 days centered around periapsis
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