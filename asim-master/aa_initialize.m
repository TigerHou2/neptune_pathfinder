% aa_initialize.m 
%   Initialize all the matrices used in asim to support compiling and
%   autocoding to C
%
% Author:   Tiger Hou
% Created:  02/18/2020
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
%   in - data structure, misc, input data structure
%   
% Output:
%   dat - data strucutre, misc, initialized trajectory data structure (NaNed to
%   length of the time vector) 


function [dat, veh] = aa_initialize(in) %#codegen

%% Vehicle data structure
veh = define_veh;
% Parameters
veh.p.m_jettison = in.v.mp.m_jettison;
% Non-integrated states
veh.s.not_jettisoned = true; % no jettison event has occured
veh.s.compute_trim = uint8(1); % enable static trim calculation
veh.s.bank = in.s.traj.bank_ini; % rad
veh.s.aoa = in.s.traj.aoa_ini; % rad
veh.s.ssa = in.s.traj.ssa_ini; % rad
veh.s.area_ref = in.v.aero.area_ref; % m^2


%% Data storage
% Determine storage array size
data_t_step = 1/in.s.data_rate;
t_vec = (in.s.traj.t_ini : data_t_step : in.s.traj.t_max)';
t_length = length(t_vec)+1; % +1 accomdates terminal point

% Trajectory
dat.traj.pos_ii = nan(t_length,3); % Inertial position
dat.traj.pos_ii_mag = nan(t_length,1); % Inertial position magnitude
dat.traj.vel_ii = nan(t_length,3); % Inertial velocity
dat.traj.vel_ii_mag = nan(t_length,1); % Inertial velocity magnitude
dat.traj.alt = nan(t_length,1); % Altitude
dat.traj.rho = nan(t_length,1); % Density
dat.traj.mass = nan(t_length,1); % Mass
dat.traj.force_ii = nan(t_length,3); % Inertial external force vector
dat.traj.thrust_ii = nan(t_length,3); % Total inertial thrust vector
dat.traj.thrust_pp = nan(t_length,3); % Total planet relative thrust vector
dat.traj.thrust_pp_mag = nan(t_length,1); % Total planet relative thrust vector magnitude
dat.traj.pos_pp = nan(t_length,3); % Planet relative position
dat.traj.pos_pp_mag = nan(t_length,1); % Planet relative position magnitude
dat.traj.V_pf_pci = nan(t_length,3); % Planet-fixed velocity
dat.traj.vel_pp = nan(t_length,3); % Planet relative velocity
dat.traj.vel_pp_mag = nan(t_length,1); % Planet relative velocity magnitude
dat.traj.gamma_ii = nan(t_length,1); % Inertial flight path angle
dat.traj.gamma_pp = nan(t_length,1); % Planet relative flight path angle
dat.traj.g_loading = nan(t_length,1); % G-loading
dat.traj.mach = nan(t_length,1); % Mach, nd
dat.traj.aoa = nan(t_length,1); % angle-of-attack, rad
dat.traj.bank = nan(t_length,1); % bank, rad
dat.traj.bank_rate = nan(t_length,1); % bank rate, rad/s
dat.traj.bank_accel = nan(t_length,1); % bank acceleration, rad/s/s
dat.traj.ssa = nan(t_length,1); % sideslip angle, rad
dat.traj.cmd_aoa = nan(t_length,1); % commanded angle-of-attack, rad
dat.traj.cmd_bank = nan(t_length,1); % commanded bank, rad
dat.traj.cl = nan(t_length,1); % lift coefficient
dat.traj.cd = nan(t_length,1); % drag coefficient
dat.traj.cd_decel = nan(t_length,1); % drag coefficient
dat.traj.LoD = nan(t_length,1); % L/D
dat.traj.gravity_ii = nan(t_length,3);
dat.traj.drag_ii = nan(t_length,3);
dat.traj.lift_ii = nan(t_length,3);
dat.traj.drag_ii_mag = nan(t_length,1);
dat.traj.lift_ii_mag = nan(t_length,1);
dat.traj.decel_drag_ii = nan(t_length,3);
dat.traj.q = nan(t_length,1);
dat.traj.azi_pp = nan(t_length,1);
dat.traj.downrange = nan(t_length,1);
dat.traj.crossrange = nan(t_length,1);
dat.traj.heat_rate = nan(t_length,1);
dat.traj.lat = nan(t_length,1);
dat.traj.lon = nan(t_length,1);
dat.traj.cg = nan(t_length,3);
dat.traj.time = nan(t_length,1); % Time

% Vehicle
dat.veh.area_ref = nan(t_length,1); % m^2

% Navigation
dat.nav.r_pci = nan(t_length,3);
dat.nav.v_inrtl_pci = nan(t_length,3);
dat.nav.a_sens_pci = nan(t_length,3);
dat.nav.rva_error = nan(t_length,9);
dat.nav.r_pcpf = nan(t_length,3);
dat.nav.v_pf_pci = nan(t_length,3);
dat.nav.v_pf_pcpf = nan(t_length,3);
dat.nav.a_sens_pcpf = nan(t_length,3);
dat.nav.lat_gd = nan(t_length,1);
dat.nav.lon = nan(t_length,1);
dat.nav.alt = nan(t_length,1);
dat.nav.dt = nan(t_length,1);
dat.nav.t = nan(t_length,1);
dat.nav.t_ini = nan(t_length,1);

% Control
dat.ctrl.deploy_decel = nan(t_length,1);
dat.ctrl.jettison = nan(t_length,1);

% MSL-prime guidance
dat.g.msl.tgt_overflown = nan(t_length,1);
dat.g.msl.phase = nan(t_length,1);
dat.g.msl.next_step = nan(t_length,1);
dat.g.msl.cmd_bank_sign = nan(t_length,1);
dat.g.msl.alt_rate = nan(t_length,1);
dat.g.msl.ang_to_tgt = nan(t_length,1);
dat.g.msl.cmd_bank = nan(t_length,1);
dat.g.msl.cmd_lodv = nan(t_length,1);
dat.g.msl.D_mag = nan(t_length,1);
dat.g.msl.lat_ang = nan(t_length,1);
dat.g.msl.lat_db = nan(t_length,1);
dat.g.msl.lod_work = nan(t_length,1);
dat.g.msl.lodv_lim_work = nan(t_length,1);
dat.g.msl.range_final = nan(t_length,1);
dat.g.msl.range_to_tgt = nan(t_length,1);
dat.g.msl.ref_alt_rate = nan(t_length,1);
dat.g.msl.ref_D_mag = nan(t_length,1);
dat.g.msl.ref_f1 = nan(t_length,1);
dat.g.msl.ref_f2 = nan(t_length,1);
dat.g.msl.ref_f3 = nan(t_length,1);
dat.g.msl.ref_range = nan(t_length,1);
dat.g.msl.tgt_transfer_ang = nan(t_length,1);
dat.g.msl.time = nan(t_length,1);
dat.g.msl.time_init = nan(t_length,1);
dat.g.msl.V_mag = nan(t_length,1);
dat.g.msl.V_norm_sq = nan(t_length,1);
dat.g.msl.U_E_pole = nan(t_length,3);
dat.g.msl.U_norm_traj = nan(t_length,3);
dat.g.msl.U_tgt = nan(t_length,3);
dat.g.msl.U_tgt_init = nan(t_length,3);
dat.g.msl.U_tgt_east = nan(t_length,3);
dat.g.msl.U_tgt_eq = nan(t_length,3);
dat.g.msl.V_work = nan(t_length,3);

% SEJ-A guidance
dat.g.sej_a.jettison = nan(t_length,2);
dat.g.sej_a.phase = nan(t_length,1);
dat.g.sej_a.next_step = nan(t_length,1);
dat.g.sej_a.iter = nan(t_length,1);
dat.g.sej_a.A_mag = nan(t_length,1);
dat.g.sej_a.K_dens = nan(t_length,1);
dat.g.sej_a.V_pf_mag = nan(t_length,1);
dat.g.sej_a.t_inc = nan(t_length,1);
dat.g.sej_a.tj_curr = nan(t_length,1);
dat.g.sej_a.tj_next = nan(t_length,1);
dat.g.sej_a.tj = nan(t_length,2);
dat.g.sej_a.ae = nan(t_length,2);
dat.g.sej_a.r_ap = nan(t_length,2);
dat.g.sej_a.delta_ap = nan(t_length,2);

% DMA-CV guidance
dat.g.dma_cv.phase = nan(t_length,1);
dat.g.dma_cv.next_step = nan(t_length,1);
dat.g.dma_cv.iter = nan(t_length,1);
dat.g.dma_cv.A_mag = nan(t_length,1);
dat.g.dma_cv.K_dens = nan(t_length,1);
dat.g.dma_cv.V_pf_mag = nan(t_length,1);
dat.g.dma_cv.t_inc = nan(t_length,1);
dat.g.dma_cv.cmd_area_ref = nan(t_length,1);
dat.g.dma_cv.ra_next = nan(t_length,1);
dat.g.dma_cv.ra = nan(t_length,2);
dat.g.dma_cv.ae = nan(t_length,2);
dat.g.dma_cv.r_ap = nan(t_length,2);
dat.g.dma_cv.delta_ap = nan(t_length,2);

% SEJ-E guidance
dat.g.sej_e.jettison = nan(t_length,1);
dat.g.sej_e.phase = nan(t_length,1);
dat.g.sej_e.next_step = nan(t_length,1);
dat.g.sej_e.iter = nan(t_length,1);
dat.g.sej_e.A_mag = nan(t_length,1);
dat.g.sej_e.K_dens = nan(t_length,1);
dat.g.sej_e.dens_est = nan(t_length,1);
dat.g.sej_e.V_pf_mag = nan(t_length,1);
dat.g.sej_e.t_inc = nan(t_length,1);
dat.g.sej_e.trust_region = nan(t_length,1);
dat.g.sej_e.ngood = nan(t_length,1);
dat.g.sej_e.t_j_curr = nan(t_length,1);
dat.g.sej_e.range_to_tgt = nan(t_length,1);
dat.g.sej_e.flight_range = nan(t_length,1);
dat.g.sej_e.delta_range = nan(t_length,1);
dat.g.sej_e.v_chute = nan(t_length,1);
dat.g.sej_e.deploy_decel = nan(t_length,1);

% BANK-A guidance
dat.g.bank_a.fpa = nan(t_length,1);
dat.g.bank_a.fpa_0 = nan(t_length,1);
dat.g.bank_a.x = nan(t_length,1);
dat.g.bank_a.y = nan(t_length,1);
dat.g.bank_a.s = nan(t_length,1);
dat.g.bank_a.s_f = nan(t_length,1);
dat.g.bank_a.A = nan(t_length,1);
dat.g.bank_a.E = nan(t_length,1);
dat.g.bank_a.cmd_bank = nan(t_length,1);
dat.g.bank_a.k = nan(t_length,1);
dat.g.bank_a.yb = nan(t_length,1);
dat.g.bank_a.rho = nan(t_length,1);

% PATHFINDER guidance
dat.g.pathfinder.cmd_bank = nan(t_length,1);
dat.g.pathfinder.fpa = nan(t_length,1);
dat.g.pathfinder.rho_est = nan(t_length,1);
dat.g.pathfinder.t = nan(t_length,1);
dat.g.pathfinder.t_ini = nan(t_length,1);
dat.g.pathfinder.R_pci = nan(t_length,3);
dat.g.pathfinder.V_inrtl_pci = nan(t_length,3);
dat.g.pathfinder.V_pf_pci = nan(t_length,3);
dat.g.pathfinder.A_sens_pci = nan(t_length,3);
dat.g.pathfinder.V_inrtl_pci_est = nan(t_length,3);
dat.g.pathfinder.V_pf_pci_est = nan(t_length,3);
dat.g.pathfinder.R_pci_est = nan(t_length,3);
dat.g.pathfinder.dt = nan(t_length,1);
dat.g.pathfinder.cd = nan(t_length,1);

% Termination
dat.term.cond = uint8(zeros(10,1));


end % aa_initialize()

