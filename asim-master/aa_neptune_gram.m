%% aa_neptune_gram.m
%
% Author:   Tiger Hou
%
% DESCRIPTION:
%   - This file creates the Neptune-GRAM model from a source paper. 
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
