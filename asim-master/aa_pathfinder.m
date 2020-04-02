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
alpha 0.5
plot3(pos(:,1), pos(:,2), pos(:,3),'LineWidth',2.5,'Color','Red')

% view(cross(r,v))
view([1,1,1])
pbaspect([1 1 1])
axis equal
grid on
hold off

%% Plot Density Estimates
% uses km and heights are w.r.t. the 1-bar reference altitude (25,264 km)
%
% NOTE: We find that the atm data given in asim starts from 24,764 km
%         which is 500 km below 25,264 km
%       Therefore my guess is that 25,264 km is the ZERO-bar reference
%         while 24,764 is the actual 1-bar reference.

size = 10;

figure(100)
scatter(test.g.pathfinder.rho_est(1:idx),...
       vecnorm(test.g.pathfinder.R_pci(1:idx,:),2,2)/1000-24764,...
       size,'filled')
grid(gca,'minor')
grid on
set(gca,'XScale','log');
xlabel('Density, $kg/m^3$')
ylabel('Height, km')

% Plot Density Estiamte Error

nep_atm = aa_make_nep_atm;

hold on
semilogx(nep_atm(:,2),nep_atm(:,1)/1e3) % nominal
hold off
title('True Density vs. Estimated Density')
legend('Estimated Density','True Density')
latexify

err = zeros(idx,1);
height = zeros(idx,1);

h_nom = nep_atm(:,1)/1e3;

idx_lim = length(h_nom);

for i = 1:idx
    height(i) = vecnorm(test.g.pathfinder.R_pci(i,:),2,2)/1000-24764;
    [~,index] = min(abs(height(i)-h_nom));
    index_shift = sign(height(i)-h_nom(index));
    if (index == 1 && index_shift == -1) || ...
       (index == idx_lim && index_shift == 1) || ...
       (index_shift == 0)
        err(i) = abs(test.g.pathfinder.rho_est(i) - nep_atm(index,2)) / ...
                 nep_atm(index,2);
        continue
    end
%     r1 = nep_atm(index,2);
%     r2 = nep_atm(index+index_shift,2);
%     h1 = h_nom(index);
%     h2 = h_nom(index+index_shift);
%     a = (r2-r1)/(h2-h1);
%     b = (r1*h2-r2*h1)/(h2-h1);
%     rho_nom = a*height(i) + b;
    rho_nom = ( (height(i)-h_nom(index))*nep_atm(index+index_shift,2) + ...
                (h_nom(index+index_shift)-height(i))*nep_atm(index,2) ) / ...
              ( h_nom(index+index_shift) - h_nom(index) );
    err(i) = abs(test.g.pathfinder.rho_est(i) - rho_nom) / rho_nom;
end

figure(101)
scatter(err,height,size,'filled')
title('Density Estimate Error vs. Height')
grid(gca,'minor')
grid on
% set(gca,'XScale','log');
xlabel('Relative Error')
ylabel('Height, km')
latexify

%% Plot Altitude as a Function of Time
figure(102)
plot(test.g.pathfinder.t(1:idx),...
     vecnorm(test.g.pathfinder.R_pci(1:idx,:),2,2)/1000-24764)
title('Vehicle Flight Path')
grid(gca,'minor')
grid on
xlabel('Time, s')
ylabel('Altitude, km')
latexify