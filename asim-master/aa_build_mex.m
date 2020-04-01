% aa_build_mex.m 
%   Generate compiled mex file of simulation
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
%   None
%   
% Output:
%   None

function [] = aa_build_mex()

%% Build MEX file

fprintf('Aeroassist Simulation Code Builder');
fprintf('\n\nBuilding MEX code...\n');

tic; % start timer

% Define input arguments (input structure)
args = { aa_define_in };

% Create custom configuration options file
config = coder.config;
config.DynamicMemoryAllocation = 'AllVariableSizeArrays'; % Allow variable size arrays

% Build MEX file
codegen -config config -report aa_main.m -args args

fprintf('...done. Elapsed time: %6.2fs\n', toc);


end % aa_build_mex()
