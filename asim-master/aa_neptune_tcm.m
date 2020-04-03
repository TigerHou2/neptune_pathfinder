%% neptune_tcm.m
%
% Author: Tiger Hou
%
% Description:
%   Simulates a s/c approaching Neptune on a hyperbolic orbit.
%   The s/c makes two TCM maneuvers:
%       TCM-1: slows down s/c while maintaining same EFPA
%       TCM-2: adjusts EFPA to a desired value.
%
%
%% Initialization

close all hidden
clear;clc

setup
neptune = get_planetary_data(8,2);

efpa = deg2rad(-14.5);

r0 = [25264000; 0; 0];
v0 = [-3534.21096; 28783.8384; 0];

visualize = false;

if visualize

    disp_orbit(r0,v0,neptune.mu,30,0.02,0,'red')

    hold on
    [x,y,z] = sphere;
    x = x*neptune.r_m;
    y = y*neptune.r_m;
    z = z*neptune.r_m;
    nep_disp = surf(x,y,z);
    alpha(nep_disp,0.5)
    hold off

    view([1,1,1])
    
end


%% TCM-1


%% TCM-2