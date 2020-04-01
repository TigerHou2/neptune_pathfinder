% aa_define_g_bank_a.m 
%   Define MSL-prime guidance state structure.
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
%   none
%   
% Output:
%   g_msl - struct(1), multi, empty guidance data structure

function [g_msl] = aa_define_g_bank_a()
%#codegen


%% Parameter data structure
  
p = struct( ... 
    'mass', double(0), ...  % kg, spacecraft initial mass
    'area_ref', double(0), ... % m^2, drag
    'fpa_e', double(0), ... % rad, entry flight path angle
    'fpa_f', double(0), ... % rad, exit flight path angle
    'beta', double(0), ...  % m^-1, inverse of atmosphere scale height
    'delta', double(0), ... % nd, non-dimensional entry speed parameter
    'eps', double(0), ...   % nd, non-dimensional entry altitude parameter
    'x_f', double(0), ...   % nd, non-dimensional exit speed parameter
    'y_f', double(0), ...   % nd, non-dimensional exit altitude parameter
    'R', double(0), ...     % m, radius of atmospheric entry
    'Ve', double(0), ...    % m/s, velocity at atmospheric entry
    'tgt_ap', double(0), ...% m, target apoapsis
    'tgt_pe', double(0), ...% m, target periapsis
    'cl', double(0), ...    % nd, lift coefficient
    'cd', double(0), ...    % nd, drag coefficient
    'rho_e', double(0) ... % kg/m^3, density at entry interface
...%     'omega', double(zeros(3,1)) ... % rad/s, planetary rotation rate
    );  


%% States data structure

s = struct( ...
    'fpa', double(0), ...
    'fpa_0', double(0), ...
    'x', double(0), ...
    'y', double(0), ...
    's', double(0), ...
    's_f', double(0), ...
    'k', double(0), ...
    'A', double(0), ...
    'E', double(0), ...
    'cmd_bank', double(0), ...
    'yb', double(0), ...
    'rho', double(0) ...
...%     'r_pf', double(zeros(3,1)), ...
...%     'v_pf', double(zeros(3,1)) ...
    );


%% States data structure

i = struct( ...
    'r_pci', double(zeros(3,1)), ... 
    'v_pci', double(zeros(3,1)), ...
    'a_pci', double(zeros(3,1)) ...
    );
    


%% Construct combined input data structure

g_msl = struct( ...
    'i', i, ...    
    's', s, ...
    'p', p ...
    );

end % aa_define_g_bank_a()
