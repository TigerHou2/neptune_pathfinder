%% aa_pathfinder
%
% Author:   Tiger Hou
%
% DESCRIPTION
%   - This script simulates a pathfinder entering Neptune prior to main
%       vehicle aerocapture. The pathfinder vehicle collects atmospheric
%       data and sends the data to the main vehicle, which gathers info
%       and improves its trajectory.
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


%% Plot Density Estimates
% uses km and heights are w.r.t. the 1-bar reference altitude (25,264 km)
%
% NOTE: We find that the atm data given in asim starts from 24,764 km
%         which is 500 km below 25,264 km
%       Therefore my guess is that 25,264 km is the ZERO-bar reference
%         while 24,764 is the actual 1-bar reference.

figure(100)
hold on

% Plot Neptune-GRAM Fminmax bounds
nep_atm = aa_make_nep_atm;
semilogx(nep_atm(:,2),nep_atm(:,1)/1e3,'--b','LineWidth',2) % nominal
semilogx(nep_atm(:,2).*(10.^(nep_atm(:,1)/8e5)),nep_atm(:,1)/1e3,'--r','LineWidth',2) %Fminmax= 1
semilogx(nep_atm(:,2)./(10.^(nep_atm(:,1)/4e5)),nep_atm(:,1)/1e3,'--g','LineWidth',2) %Fminmax=-1

% Plot polynomial fit of density
rho = test.g.pathfinder.rho_est(1:idx);
rho_log = log10(rho);
alt = vecnorm(test.g.pathfinder.R_pci_est(1:idx,:),2,2)/1000-24764;
rho_log(alt<100) = [];
rho(alt<100) = [];
alt(alt<100) = [];
p = polyfit(alt,rho_log,3);
xx = polyval(p,alt);
semilogx(10.^xx,alt,'-m','LineWidth',2)

% Plot density measurements
scatter(test.g.pathfinder.rho_est(1:idx),...
        vecnorm(test.g.pathfinder.R_pci_est(1:idx,:),2,2)/1000-24764,...
        7,'filled','MarkerFaceAlpha',.3,'MarkerEdgeAlpha',.3)

hold off

grid(gca,'minor'); grid on
set(gca,'XScale','log');
xlabel('Density, $kg/m^3$')
ylabel('Height, km')
title('True Density vs. Estimated Density')
legend('Fminmax = 1','True Density','Fminmax = -1','Density Estimate')
latexify


%% Plot Density Errors

h_nom = nep_atm(:,1)/1e3;
err = abs( nep_atm(:,2) - 10.^(polyval(p,h_nom)) ) ./ nep_atm(:,2);

err(h_nom<100) = [];
h_nom(h_nom<100) = [];

figure(101)
hold on
plot(err,h_nom,'LineWidth',1)
hold off
title('Density Estimate Error vs. Height')
grid(gca,'minor')
grid on
set(gca,'XScale','log');
xlabel('Relative Error')
ylabel('Height, km')
latexify


%% Plot Trajectory on Globe

figure(102)

pos = test.g.pathfinder.R_pci_est(1:idx,:);

hold on

[x,y,z] = sphere;
x = x*in.p.r_m;
y = y*in.p.r_m;
z = z*in.p.r_m;
surf(x,y,z)
alpha 0.5
plot3(pos(:,1), pos(:,2), pos(:,3),'LineWidth',2.0,'Color','Red')

% view(cross(r,v))
view([1,1,1])
pbaspect([1 1 1])
axis equal
grid on
hold off


%% Plot Altitude as a Function of Time

figure(103)
plot(test.g.pathfinder.t(1:idx),...
     vecnorm(test.g.pathfinder.R_pci_est(1:idx,:),2,2)/1000-24764)
title('Vehicle Flight Path')
grid(gca,'minor')
grid on
xlabel('Time, s')
ylabel('Altitude, km')
latexify

%% Plot Acceleration as a Function of Time

figure(104)
plot(test.g.pathfinder.t(1:idx),...
     vecnorm(test.g.pathfinder.A_sens_pci(1:idx,:),2,2))
title('Vehicle Flight Path')
grid(gca,'minor')
grid on
xlabel('Time, s')
ylabel('Acceleration, $m/s^2$')
latexify
