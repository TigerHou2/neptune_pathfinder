% aa_guid_pathfinder.m 
%   Define pathfinder data collection probe guidance code
%
% Author:   Tiger Hou
% Created:  03/03/2020
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
%   i - struct(1), multi, guidance input structure
%   s - struct(1), multi, guidance state structure
%   p - struct(1), multi, guidance parameter structure
%   init_flag - logical(1), nd, first-pass initialization flag
%   
% Output:
%   s - struct(1), multi, guidance state structure
%
% Sources:
%   - [1] Mars Aerocapture Using bank Modulation

function [s] = aa_guid_pathfinder( i, s, p, init_flag )
%#codegen

%% First-pass Initialization
if init_flag
    s = init(i, s, p);
end

%% Preliminary calculations

s.dt = i.t - s.t_ini - s.t;
s.t = i.t - s.t_ini; % s, guidance time, relative to initialization time
    s.R_pci = i.R_pci;
    s.V_inrtl_pci = i.V_inrtl_pci;
    s.V_pf_pci = i.V_pf_pci;
s.A_sens_pci = i.A_sens_pci;
s.cd = i.cd;

% numerical integration of accelerometer data
s.V_inrtl_pci_est = s.V_inrtl_pci_est + (s.A_sens_pci + i.g_pci) * s.dt;
s.R_pci_est = s.R_pci_est + s.V_inrtl_pci_est * s.dt;

theta = norm(p.omega) * i.t;
L = [ cos(theta) sin(theta) 0; ...
     -sin(theta) cos(theta) 0; ...
      0          0          1];

s.V_pf_pci_est = L * ( s.V_inrtl_pci_est - cross(p.omega,s.R_pci_est) );

s.rho_est = (2*p.mass*norm(s.A_sens_pci)) ...
          / (norm(s.V_pf_pci_est)^2*p.area_ref*s.cd); % kg/m^3
s.fpa = pi/2-atan2( norm(cross(s.R_pci_est,s.V_pf_pci_est)) ...
                       , dot(s.R_pci_est,s.V_pf_pci_est) );

% s.rho_est = (2*p.mass*norm(i.A_sens_pci))/(norm(i.V_pf_pci)^2*p.area_ref*p.cd); % kg/m^3
% s.fpa = pi/2-atan2(norm(cross(i.R_pci,i.V_pf_pci)),dot(i.R_pci,i.V_pf_pci));

switch p.type
    case 1 % probe
        s.cmd_bank = 0;
    case 2 % main
        s.cmd_bank = 0; % placeholder
    otherwise
        s.cmd_bank = 0;
end

end % aa_guid_bank_test



function [ s ] = init(i, s, p)
% First-pass initialization function

%% Initialize states
s.cmd_bank = 0;
s.t_ini = i.t;

s.V_inrtl_pci_est = i.V_inrtl_pci;
s.V_pf_pci_est = i.V_pf_pci;
s.R_pci_est = i.R_pci;
        
end % init

