% setup.m 
%   Perform setup tasks for asim
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
%   none.
%
% Output:
%   none.


%% Add directories to Matlab path
% addpath(['.' filesep 'analysis']);
addpath(['.' filesep 'input']);
addpath(['.' filesep 'utils']);
addpath(['.' filesep 'define']);
addpath(['.' filesep 'data']);
addpath(['.' filesep 'guidance']);
addpath(['.' filesep 'get']);
addpath(['.' filesep 'aa_supp_fcns']);
addpath(['.' filesep 'aa-contours']);
plottools('off'); close all; % initializes plot tools but keeps it hidden