%% aa_tutorial_v1.m
%
% Author:   Tiger Hou
% Created:  02/06/2020
% Modified: 02/08/2020
%
% DESCRIPTION:
%   - This file creates the Neptune-GRAM model from a source paper, and
%       performs a single run simulation of aerocapture with a single-
%       jettison event drag modulation scheme. 
%
% SOURCES:
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies
%
% MODIFICATIONS:
%
%

%%

close all hidden
clear;clc

% run setup file to include asim paths
setup
in = aa_tutorial_v1_in();

%% Create Neptune-GRAM

% Fminmax determines the atmosphere scale factor
Fminmax = 1;
% get nominal Neptune atmosphere table
nep_atm = in.p.atm.table;
% plot neptune nominal density
figure(1)
hold on
semilogx(nep_atm(:,2),nep_atm(:,1)/1e3) % nominal
semilogx(nep_atm(:,2).*(10.^(nep_atm(:,1)/8e5)),nep_atm(:,1)/1e3) %Fminmax= 1
semilogx(nep_atm(:,2)./(10.^(nep_atm(:,1)/4e5)),nep_atm(:,1)/1e3) %Fminmax=-1
set(gca, 'XScale', 'log');
grid on
xlabel('Density, kg/m^3')
ylabel('Height, km')
hold off
% ratio provided in Fig 3(b) from [1]
LoH = 2;
% gravity of Neptune (at surface) divided by specifc heat constant
Cp = (in.p.atm.gamma * in.p.atm.R) / (in.p.atm.gamma - 1);
goCp = in.p.surface_g / Cp;
goR  = in.p.surface_g / in.p.atm.R;
% eqn.1 from [1]
% fudging the numbers a bit to match [1]
rho_ratio_pct = LoH/2/pi * sqrt(1+(LoH/2/pi)^2)...
                * (0.18*movmean(diff(nep_atm(:,4)),30) + goCp) / goR + 9;
nep_pert = [nep_atm(2:end,1), rho_ratio_pct / 100];
figure(2)
plot(nep_pert(:,1)/1e3,nep_pert(:,2))
grid on
xlabel('Height, km')
ylabel('Max Perturbation Magnitude, % of Mean')

%% Run Simulation

dat = main1_mex(in);

%% Plot Orbit

idx = sum(~isnan(dat.traj.time));

r = dat.traj.pos_ii(idx,:)';
v = dat.traj.vel_ii(idx,:)';

res = 100;
start_day = 0.1;
mu = in.p.mu;

ah = norm(r)/(2 - norm(r)*dot(v,v)/mu);
if ah >= 0
    dur = 2*pi*sqrt(ah^3/mu)/3600/24;  % orbit propagation duration [days]
else
    ap = in.v.gnc.g.p_sej_a.tgt_apoapse;
    dur = 2*pi*sqrt(ap^3/mu)/3600/24;
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