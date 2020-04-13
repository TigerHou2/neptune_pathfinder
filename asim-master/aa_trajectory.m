% aa_trajectory.m 
%   Compute forces for numerical integration
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
%   in - struct(1), multi, input data structure
%   veh - struct(1), multi, vehicle data structure
%   dat - struct(1), multi, initialized data structure
%   
% Output:
%   dat - data structure, misc, initialized data structure

function [dat] = aa_trajectory(in, veh, dat)
%#codegen

%% Initialization

% Simulation parameters
termination_flag = logical(false);
h = 1/in.s.traj.rate; % s, Time-step
sd_ratio = round(in.s.traj.rate/in.s.data_rate); % sim/data rate ratio, nd
ttvec = (in.s.traj.t_ini : h : in.s.traj.t_max)'; % Trajectory time vector, s

% Initial vehicle state
y0 = [in.s.traj.r_pci_ini; ... % PCI position vector, m
      in.s.traj.v_pci_ini; ... % inertial PCI velocity vector, m/s
      in.v.mp.m_ini; ...       % mass, kg
      in.s.traj.dr_ini; ...    % downrange, m
      in.s.traj.cr_ini; ...    % crossrange, m
      0.0 ];                   % heat load, W/m^2

% Initialize GNC data structures
[nav, nav_rnd] = aa_navigation_init(in);
guid = aa_guidance_init(in);
ctrl = control_init(in);


%% Perform Integration - Fixed-step 4th-order Runge-Kutta method

% Compute calculations of interest
i = 1; % integration counter
di = 1; % data logging counter
ti = ttvec(i); % current time
calcs_curr = traj_calcs( in, ti, y0, veh, ctrl ); % perform trim calculation
calcs_prev = calcs_curr;
y0(7,i) = y0(7,i) + calcs_curr.delta_mass;
azi_pp0 = calcs_curr.azi_pp;

% Initial call to flight computer (GNC)
nav = aa_navigation( ttvec(i), i, calcs_curr, nav_rnd, nav );
guid = aa_guidance( in, calcs_curr, ttvec(i), i, nav, guid );
ctrl = control( calcs_curr, ttvec(i), i, nav, guid, ctrl );

% Initial call to vehicle
veh.s.bank = ctrl.s.bank;
veh.s.aoa = ctrl.s.aoa;

% Store data
dat = store_dat( calcs_curr, di, ti, y0, veh, guid, nav, ctrl, dat, in );

% Initialize arrays
neq = length(y0);
N = length(ttvec);
Y1 = zeros(neq,N);
F = zeros(neq,4);

% Integration loop
Y1(:,1) = y0; % set initial conditions
for i = 2:N
        
    %% Integrate Equations of Motion
    % Perform RK4 integration (1 time step)
    ti = ttvec(i-1); % set previous time
    yi = Y1(:,i-1); % copy previous state vector
    veh.s.compute_trim = uint8(0); % disable static trim calcultion
    F(:,1) = eom_flight(ti, yi, in, azi_pp0, veh, ctrl ); % no trim calculation
    F(:,2) = eom_flight(ti+0.5*h, yi+0.5*h*F(:,1), in, azi_pp0, veh, ctrl ); % no trim calculation
    F(:,3) = eom_flight(ti+0.5*h, yi+0.5*h*F(:,2), in, azi_pp0, veh, ctrl ); % no trim calculation   
    F(:,4) = eom_flight(ti+h, yi+h*F(:,3), in, azi_pp0, veh, ctrl ); % no trim calculation
    Y1(:,i) = yi + (h/6)*(F(:,1) + 2*F(:,2) + 2*F(:,3) + F(:,4)); % set new state vector    
    % Compute state-dependent calculations of interest at current time step
    calcs_prev = calcs_curr; % store previous calculations
    veh.s.compute_trim = uint8(1); % enable static trim calcultion
    calcs_curr = traj_calcs( in, ttvec(i), Y1(:,i), veh, ctrl ); % update calculations (with trim calculation)


    %% Flight computer (GNC)
    nav = aa_navigation( ttvec(i), i, calcs_curr, nav_rnd, nav );
    guid = aa_guidance( in, calcs_curr, ttvec(i), i, nav, guid );
    ctrl = control( calcs_curr, ttvec(i), i, nav, guid, ctrl );

    
    %% Effectors (vehicle state changes)
    % [Y1, veh] = effectors( ctrl, Y1, veh ); % DOES NOT CURRENTLY EXIST
    veh.s.bank = ctrl.s.bank;
    veh.s.aoa = ctrl.s.aoa;
    if (ctrl.s.jettison == true) && (veh.s.not_jettisoned == true)
        veh.s.area_ref = guid.cmd.area_ref;
        veh.s.not_jettisoned = false;
        Y1(7,i) = Y1(7,i) - veh.p.m_jettison;
        ctrl.s.jettison = false;
    end
    Y1(4:6,i) = Y1(4:6,i) + calcs_curr.delta_V; % update velocity with any instantaneous changes
    Y1(7,i) = Y1(7,i) + calcs_curr.delta_mass; % update mass with any discrete changes

    
    %% Store data
    % Rate limit storage
    if mod(i-1,sd_ratio)==0
        di = di + 1;
        dat = store_dat( calcs_curr, di, ttvec(i), Y1(:,i), veh, guid, nav, ctrl, dat, in );    
    end

    
    %% Termination Logic
    % Check termination conditions
    [termination_flag,cond]=termination(in,calcs_curr, calcs_prev); % check termination conditions    
    if termination_flag 
        % Write final data point
        if mod(i-1,sd_ratio)~=0
            di = di + 1;
            dat = store_dat( calcs_curr, di, ttvec(i), Y1(:,i), veh, guid, nav, ctrl, dat, in );
            dat.term.cond = cond;
        end
        break; % Terminate integration
    end
    
end % integration loop (i=2:N)


end % aa_trajectory()



function dat = store_dat( calcs, i, ti, y, veh, guid, nav, ctrl, dat, in )
%% Store calculations of interest as trajectory data

    % Store trajectory data
    dat.traj.pos_ii(i,:) = calcs.pos_ii; % Inertial position
    dat.traj.pos_ii_mag(i) = calcs.pos_ii_mag; % Inertial position magnitude
    dat.traj.vel_ii(i,:) = calcs.vel_ii; % Inertial velocity
    dat.traj.vel_ii_mag(i) = calcs.vel_ii_mag; % Inertial velocity magnitude
    dat.traj.alt(i) = calcs.alt; % Altitude
    dat.traj.rho(i) = calcs.rho; % Density
    dat.traj.mass(i) = calcs.mass + calcs.delta_mass; % Mass
    dat.traj.force_ii(i,:) = calcs.force_ii; % Inertial external force vector
    dat.traj.thrust_ii(i,:) = calcs.thrust_ii; % Total inertial thrust vector
    dat.traj.thrust_pp(i,:) = calcs.thrust_pp; % Total planet relative thrust vector
    dat.traj.thrust_pp_mag(i) = calcs.thrust_pp_mag; %Total planet relative thrust vector magnitude
    dat.traj.pos_pp(i,:) = calcs.pos_pp; % Planet relative position
    dat.traj.pos_pp_mag(i) = calcs.pos_pp_mag; % Planet relative position magnitude
    dat.traj.V_pf_pci(i,:) = calcs.V_pf_pci; % Planet-fixed velocity
    dat.traj.vel_pp(i,:) = calcs.vel_pp; % Planet relative velocity
    dat.traj.vel_pp_mag(i) = calcs.vel_pp_mag; % Planet relative velocity magnitude
    dat.traj.gamma_ii(i) = calcs.gamma_ii; % Inertial flight path angle
    dat.traj.gamma_pp(i) = calcs.gamma_pp; % Planet relative flight path angle
    dat.traj.g_loading(i) = calcs.g_loading; % G-loading, Earth g
    dat.traj.mach(i) = calcs.mach; % Mach number, nd
    dat.traj.aoa(i) = calcs.aoa; % angle-of-attack, rad
    dat.traj.bank(i) = calcs.bank; % bank angle, rad
    dat.traj.bank_rate(i) = ctrl.s.bank_rate; % bank angle rate, rad/s
    dat.traj.bank_accel(i) = ctrl.s.bank_accel; % bank angle acceleration, rad/s/s
    dat.traj.ssa(i) = calcs.ssa; % sideslip angle, rad
    dat.traj.cmd_aoa(i) = guid.cmd.aoa; % commanded angle-of-attack, rad
    dat.traj.cmd_bank(i) = guid.cmd.bank; % commanded bank angle, rad
    dat.traj.gravity_ii(i,:) = calcs.gravity_ii; % gravity force vector, N
    dat.traj.drag_ii(i,:) = calcs.drag_ii; % drag force vector, N
    dat.traj.lift_ii(i,:) = calcs.lift_ii; % lift force vector, N
    dat.traj.drag_ii_mag(i) = calcs.drag_ii_mag; % drag force magnitude, N
    dat.traj.lift_ii_mag(i) = calcs.lift_ii_mag; % lift force magnitude, N
    dat.traj.decel_drag_ii(i,:) = calcs.decel_drag_ii; % drag force vector, N
    dat.traj.cl(i) = calcs.cl; % lift coefficient, nd
    dat.traj.cd(i) = calcs.cd; % drag coefficient, nd
    dat.traj.cd_decel(i) = calcs.cd_decel; % decelerator drag coefficient, nd
    dat.traj.LoD(i) = calcs.LoD; % lift-to-drag ratio, nd
    dat.traj.q(i) = calcs.q; % dynamic pressure, N/m^2
    dat.traj.azi_pp(i) = calcs.azi_pp;
    dat.traj.downrange(i) = y(8); % downrange, m
    dat.traj.crossrange(i) = y(9); % crossrange, m
    dat.traj.heat_rate(i) = calcs.heat_rate; % stagnation-point heat rate, W/m^2
    dat.traj.lat(i) = calcs.lat; % latitude (centric or detic), rad
    dat.traj.lon(i) = calcs.lon; % longiude, rad
    dat.traj.cg(i,:) = calcs.cg; % center of gravity position, m
    dat.traj.time(i) = ti; % Time
    
    % Vehicle data
    dat.veh.area_ref(i) = veh.s.area_ref; % m^2
    
    % Navigation data
    dat.nav.r_pci(i,:) = nav.s.r_pci;
    dat.nav.v_inrtl_pci(i,:) = nav.s.v_inrtl_pci;
    dat.nav.a_sens_pci(i,:) = nav.s.a_sens_pci;
    dat.nav.rva_error(i,:) = nav.s.rva_error;
    dat.nav.r_pcpf(i,:) = nav.s.r_pcpf;
    dat.nav.v_pf_pci(i,:) = nav.s.v_pf_pci;
    dat.nav.v_pf_pcpf(i,:) = nav.s.v_pf_pcpf;
    dat.nav.a_sens_pcpf(i,:) = nav.s.a_sens_pcpf;
    dat.nav.lat_gd(i) = nav.s.lat_gd;
    dat.nav.lon(i) = nav.s.lon;
    dat.nav.alt(i) = nav.s.alt;
    dat.nav.dt(i) = nav.s.dt;
    dat.nav.t(i) = nav.s.t;
    dat.nav.t_ini(i) = nav.s.t_ini;
    
    % Control data
    dat.ctrl.deploy_decel(i) = ctrl.s.deploy_decel;
    dat.ctrl.jettison(i) = ctrl.s.jettison;
    
    % Store MSL-prime guidance data
    dat.g.msl.tgt_overflown(i) = guid.msl.s.tgt_overflown;
    dat.g.msl.phase(i) = guid.msl.s.phase; 
    dat.g.msl.next_step(i) = guid.msl.s.next_step; 
    dat.g.msl.cmd_bank_sign(i) = guid.msl.s.cmd_bank_sign; 
    dat.g.msl.alt_rate(i) = guid.msl.s.alt_rate; 
    dat.g.msl.ang_to_tgt(i) = guid.msl.s.ang_to_tgt; 
    dat.g.msl.cmd_bank(i) = guid.msl.s.cmd_bank; 
    dat.g.msl.cmd_lodv(i) = guid.msl.s.cmd_lodv; 
    dat.g.msl.D_mag(i) = guid.msl.s.D_mag; 
    dat.g.msl.lat_ang(i) = guid.msl.s.lat_ang; 
    dat.g.msl.lat_db(i) = guid.msl.s.lat_db; 
    dat.g.msl.lod_work(i) = guid.msl.s.lod_work; 
    dat.g.msl.lodv_lim_work(i) = guid.msl.s.lodv_lim_work; 
    dat.g.msl.range_final(i) = guid.msl.s.range_final; 
    dat.g.msl.range_to_tgt(i) = guid.msl.s.range_to_tgt; 
    dat.g.msl.ref_alt_rate(i) = guid.msl.s.ref_alt_rate; 
    dat.g.msl.ref_D_mag(i) = guid.msl.s.ref_D_mag; 
    dat.g.msl.ref_f1(i) = guid.msl.s.ref_f1; 
    dat.g.msl.ref_f2(i) = guid.msl.s.ref_f2; 
    dat.g.msl.ref_f3(i) = guid.msl.s.ref_f3; 
    dat.g.msl.ref_range(i) = guid.msl.s.ref_range; 
    dat.g.msl.tgt_transfer_ang(i) = guid.msl.s.tgt_transfer_ang; 
    dat.g.msl.time(i) = guid.msl.s.time; 
    dat.g.msl.time_init(i) = guid.msl.s.time_init; 
    dat.g.msl.V_mag(i) = guid.msl.s.V_mag; 
    dat.g.msl.V_norm_sq(i) = guid.msl.s.V_norm_sq; 
    dat.g.msl.U_E_pole(i,:) = guid.msl.s.U_E_pole; 
    dat.g.msl.U_norm_traj(i,:) = guid.msl.s.U_norm_traj; 
    dat.g.msl.U_tgt(i,:) = guid.msl.s.U_tgt; 
    dat.g.msl.U_tgt_init(i,:) = guid.msl.s.U_tgt_init; 
    dat.g.msl.U_tgt_east(i,:) = guid.msl.s.U_tgt_east; 
    dat.g.msl.U_tgt_eq(i,:) = guid.msl.s.U_tgt_eq; 
    dat.g.msl.V_work(i,:) = guid.msl.s.V_work; 

    % Store SEJ-A guidance data
    dat.g.sej_a.jettison(i,1) = guid.sej_a.s.jettison(1);
    dat.g.sej_a.jettison(i,2) = guid.sej_a.s.jettison(2);
    dat.g.sej_a.phase(i) = guid.sej_a.s.phase;
    dat.g.sej_a.next_step(i) = guid.sej_a.s.next_step;
    dat.g.sej_a.iter(i) = guid.sej_a.s.iter;
    dat.g.sej_a.A_mag(i) = guid.sej_a.s.A_mag;
    dat.g.sej_a.K_dens(i) = guid.sej_a.s.K_dens;
    dat.g.sej_a.V_pf_mag(i) = guid.sej_a.s.V_pf_mag;
    dat.g.sej_a.t_inc(i) = guid.sej_a.s.t_inc;
    dat.g.sej_a.tj_curr(i) = guid.sej_a.s.tj_curr;
    dat.g.sej_a.tj_next(i) = guid.sej_a.s.tj_next;
    dat.g.sej_a.tj(i,:) = guid.sej_a.s.tj;
    dat.g.sej_a.ae(i,:) = guid.sej_a.s.ae;
    dat.g.sej_a.r_ap(i,:) = guid.sej_a.s.r_ap;
    dat.g.sej_a.delta_ap(i,:) = guid.sej_a.s.delta_ap;

    % Store DMA-CV guidance data
    dat.g.dma_cv.phase(i) = guid.dma_cv.s.phase;
    dat.g.dma_cv.next_step(i) = guid.dma_cv.s.next_step;
    dat.g.dma_cv.iter(i) = guid.dma_cv.s.iter;
    dat.g.dma_cv.A_mag(i) = guid.dma_cv.s.A_mag;
    dat.g.dma_cv.K_dens(i) = guid.dma_cv.s.K_dens;
    dat.g.dma_cv.V_pf_mag(i) = guid.dma_cv.s.V_pf_mag;
    dat.g.dma_cv.t_inc(i) = guid.dma_cv.s.t_inc;
    dat.g.dma_cv.cmd_area_ref(i) = guid.dma_cv.s.cmd_area_ref;
    dat.g.dma_cv.ra_next(i) = guid.dma_cv.s.ra_next;
    dat.g.dma_cv.ra(i,:) = guid.dma_cv.s.ra;
    dat.g.dma_cv.ae(i,:) = guid.dma_cv.s.ae;
    dat.g.dma_cv.r_ap(i,:) = guid.dma_cv.s.r_ap;
    dat.g.dma_cv.delta_ap(i,:) = guid.dma_cv.s.delta_ap;   
    
    % Store SEJ-E guidance data
    dat.g.sej_e.jettison(i) = guid.sej_e.s.jettison;
    dat.g.sej_e.phase(i) = guid.sej_e.s.phase;
    dat.g.sej_e.next_step(i) = guid.sej_e.s.next_step;
    dat.g.sej_e.iter(i) = guid.sej_e.s.iter;
    dat.g.sej_e.A_mag(i) = guid.sej_e.s.A_mag;
    dat.g.sej_e.K_dens(i) = guid.sej_e.s.K_dens;
    dat.g.sej_e.dens_est(i) = guid.sej_e.s.dens_est;
    dat.g.sej_e.V_pf_mag(i) = guid.sej_e.s.V_pf_mag;
    dat.g.sej_e.t_inc(i) = guid.sej_e.s.t_inc;
    dat.g.sej_e.trust_region(i) = guid.sej_e.s.trust_region;
    dat.g.sej_e.ngood(i) = guid.sej_e.s.ngood;
    dat.g.sej_e.t_j_curr(i) = guid.sej_e.s.t_j_curr;
    dat.g.sej_e.range_to_tgt(i) = guid.sej_e.s.range_to_tgt;
    dat.g.sej_e.flight_range(i) = guid.sej_e.s.flight_range;
    dat.g.sej_e.delta_range(i) = guid.sej_e.s.delta_range;
    dat.g.sej_e.v_chute(i) = guid.sej_e.s.v_chute;
    dat.g.sej_e.deploy_decel(i) = guid.sej_e.s.deploy_decel;
    
    % Store BANK-A guidance data
    dat.g.bank_a.fpa(i) = guid.bank_a.s.fpa;
    dat.g.bank_a.fpa_0(i) = guid.bank_a.s.fpa_0;
    dat.g.bank_a.x(i) = guid.bank_a.s.x;
    dat.g.bank_a.y(i) = guid.bank_a.s.y;
    dat.g.bank_a.s(i) = guid.bank_a.s.s;
    dat.g.bank_a.s_f(i) = guid.bank_a.s.s_f;
    dat.g.bank_a.k(i) = guid.bank_a.s.k;
    dat.g.bank_a.A(i) = guid.bank_a.s.A;
    dat.g.bank_a.E(i) = guid.bank_a.s.E;
    dat.g.bank_a.cmd_bank(i) = guid.bank_a.s.cmd_bank;
    dat.g.bank_a.yb(i) = guid.bank_a.s.yb;
    dat.g.bank_a.rho(i) = guid.bank_a.s.rho;
    
    % Store PATHFINDER guidance data
    dat.g.pathfinder.cmd_bank(i) = guid.pathfinder.s.cmd_bank;
    dat.g.pathfinder.fpa(i) = guid.pathfinder.s.fpa;
    dat.g.pathfinder.rho_est(i) = guid.pathfinder.s.rho_est;
    dat.g.pathfinder.t(i) = guid.pathfinder.s.t;
    dat.g.pathfinder.t_ini(i) = guid.pathfinder.s.t_ini;
    dat.g.pathfinder.R_pci(i,:) = guid.pathfinder.s.R_pci;
    dat.g.pathfinder.V_inrtl_pci(i,:) = guid.pathfinder.s.V_inrtl_pci;
    dat.g.pathfinder.V_pf_pci(i,:) = guid.pathfinder.s.V_pf_pci;
    dat.g.pathfinder.A_sens_pci(i,:) = guid.pathfinder.s.A_sens_pci;
    dat.g.pathfinder.V_inrtl_pci_est(i,:) = guid.pathfinder.s.V_inrtl_pci_est;
    dat.g.pathfinder.V_pf_pci_est(i,:) = guid.pathfinder.s.V_pf_pci_est;
    dat.g.pathfinder.R_pci_est(i,:) = guid.pathfinder.s.R_pci_est;
    dat.g.pathfinder.dt(i) = guid.pathfinder.s.dt;
    dat.g.pathfinder.cd(i) = guid.pathfinder.s.cd;
    

end % store_dat()
