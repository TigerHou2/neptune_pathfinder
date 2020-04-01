% control_init.m 
%   Initialize control data structure
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
%   in - data structure, misc, input data structure
%   
% Output:
%   ctrl - data structure, misc, control data structure with parameter
%       values set

function [ctrl] = control_init( in )
%#codegen

%% Create data structure
ctrl = define_ctrl;

%% Initialize parameters
ctrl.p = in.v.gnc.c.p;

%% Initialize states
% Time state
ctrl.s.t_ini = in.s.traj.t_ini; % s
ctrl.s.t_prev = in.s.traj.t_ini; % s
ctrl.s.t = in.s.traj.t_ini; % s
ctrl.s.dt = 0; % s
ctrl.s.bank_accel = 0; % rad/s^2
ctrl.s.bank_rate = in.v.gnc.g.p.const_bank_rate; % rad/s
ctrl.s.bank = in.v.gnc.g.p.const_bank; % rad
ctrl.s.area_ref = in.v.aero.area_ref; % m^2
ctrl.s.sc_ratio = round(in.s.traj.rate/ctrl.p.rate); % sim/control rate ratio, nd

ctrl.s.jettison = false;
ctrl.s.ignition = false;
ctrl.s.deploy_decel = false;

end % control_init