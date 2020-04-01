%% aa_drag_mod_v1
%
% Author:   Tiger Hou
% Created:  02/07/2020
% Modified: 02/09/2020
%
% DESCRIPTION
%   - This script simulates aerocapture at Neptune using drag modulation.
%       The parameters varied include B2/B1. initial velocity, and FPA.
%   - Final orbits within a certain apoapsis tolerance are plotted with
%       FPA as a function of B2/B1 and initial velocity.
%
% SOURCES:
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies
%   - [3] Neptune Aerocapture Systems Analysis
%

%% Initialization

close all hidden
clear;clc
setup
in = aa_drag_mod_v1_in();    % load the input file

% create Neptune atmosphere dispersion model
% (currently we are not using it)
% nep_atm = aa_make_nep_atm(in);

%% Data Saving Options

% get filename for data file
opts.Interpreter = 'tex';
fName = inputdlg( '\fontsize{12} Enter data file name', ...
                  'User Input', [1,50], ...
                 {'data_file'}, opts);
if isempty(fName)
    warning('No file name provided. Using default name "data_file.mat".');
    fName = 'data_file';
else
    fName = fName{1};
end

%% Run Simulation

% orbiting body gravitational parameter
mu = in.p.mu;

% simulation ranges
beta_ratio = 7:0.5:12;
v_init     = (25:0.5:35)*1e3;
gammas     = -7.5:0.5:-5.0;

% get size of each dimension
br_len = length(beta_ratio);
vi_len = length(v_init);
ga_len = length(gammas);

% stores boolean values of whether orbit converged
apoapsis_result = zeros(br_len*vi_len*ga_len,1);
tol_range = ...
    [in.v.gnc.g.p_sej_a.tgt_ap - in.v.gnc.g.p_sej_a.tol_ap, ...
     in.v.gnc.g.p_sej_a.tgt_ap + in.v.gnc.g.p_sej_a.tol_ap];

counter = 0;
datsize = br_len*vi_len*ga_len;

in_cell = cell(br_len,vi_len,ga_len);

for i = 1:br_len
    for j = 1:vi_len
        for k = 1:ga_len
            in_cell{i,j,k} = [beta_ratio(i),v_init(j),gammas(k)];
        end
    end
end

parfor i = 1:datsize
    % start simulation and increment counter
    disp(['Progress: ' num2str(i)])
    temp = in_cell{i};
    br = temp(1);
    b1 = 10;
    b2 = b1 * br;
    vi = temp(2);
    ga = temp(3);
    this_in = in;
    this_in.v.gnc.g.p_sej_a.area_ref = ...
        [this_in.v.aero.area_ref; this_in.v.mp.m_ini/(this_in.v.aero.cd*b2)]; % m^2
    this_in.s.traj.v_pci_ini = vi*[sind(ga); cosd(ga); 0];
    
    dat_temp = aa_main_mex(this_in);
    
    % calculate apoapsis of orbit
    idx = sum(~isnan(dat_temp.traj.time)); % termination index
    r = dat_temp.traj.pos_ii(idx,:)';      % termination position
    v = dat_temp.traj.vel_ii(idx,:)';      % termination velocity
    apoapsis_result(i) = norm(r)/(2 - norm(r)*dot(v,v)/mu);
end

save(['aa-contours/' fName '.mat'], ...
    'apoapsis_result', 'in', ...
    'beta_ratio', 'v_init', 'gammas', ...
    'br_len', 'vi_len', 'ga_len');

disp('')
disp('Finished!')

%% Plot Accepted Orbits

aa_corridor_surf_plot('filename',fName)
