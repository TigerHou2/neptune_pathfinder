% guidance.m
%   Calls guidance algorithms based on guidance mode flag
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
%   calcs - struct(1), multi, traj_calcs output
%   t - double(1), s, time
%   si - double(1), nd, simulation step index
%   nav - struct(1), multi, navigation data structure
%   guid - struct(1), multi, guidance data structure
%
% Output:
%   guid - struct(1), multi, guidance data structure

function [ guid ] = guidance( in, calcs, t, si, nav, guid )
%#codegen

if mod(si-1,guid.s.sg_ratio) == 0 % rate limit calls to guidance

    %% Initialize
    % Local commands
    cmd_aoa = 0; % rad
    cmd_bank = 0; % rad
    cmd_t_jettison = 9e9; % s
    cmd_area_ref = in.v.aero.area_ref; % m^2
    cmd_deploy_decel = false; % nd
    
    %% Bank angle guidance
    switch guid.p.bank_mode

        case 1 % Constant bank

            cmd_bank = guid.p.const_bank; % Bank command, rad

        case 2 % Constant bank rate 

            % Calculate bank angle based on constant bank rate
            cmd_bank = guid.p.const_bank_rate*t + guid.p.const_bank; % Bank command, rad
            % Constrain Bank Angle to be within -pi and pi
            cmd_bank = mod(cmd_bank,2*pi);
            if cmd_bank > pi
                cmd_bank = cmd_bank - 2*pi;
            end
        
        case 3 % Velocity schedule
         
            % Requires monotonically increasing velocity domain
            len_bank = guid.p.bank_sched_len;            
            cmd_bank = interp1(guid.p.bank_schedule(1:len_bank,2),guid.p.bank_schedule(1:len_bank,1),norm(nav.s.v_inrtl_pci),'linear',0);

        case 4 % Entry: MSL guidance (Apollo Final Phase)

            % Assign inputs
            guid.msl.i.timetag = nav.s.t; % current time
            guid.msl.i.V_ocg_wrt_Eartho_icrf_ICRF = nav.s.v_inrtl_pci; % m/s, inertial velocity vector, ECI
            guid.msl.i.V_ocg_wrt_Eartho_itrf_ICRF = nav.s.v_pf_pci; % m/s, planet-relative velocity vector, ECI
            guid.msl.i.R_ocg_wrt_Eartho_ICRF = nav.s.r_pci; % m, position vector, ECI
            guid.msl.i.Asens_onb_wrt_Eartho_icrf_OB = nav.s.a_sens_pci; % m/s^2, sensed acceleration vector, body frame       

            % Call MSL guidance
            guid.msl.s = guid_msl_prime( guid.msl.i, guid.msl.s, guid.msl.p, guid.s.init_flag );

            % Assign bank command
            cmd_bank = guid.msl.s.cmd_bank; % Bank command, rad
            
           
        otherwise % No guidance - issue constant bank command

            cmd_bank = guid.p.const_bank; % Bank command, rad

          
    end % switch guid.p.bank_mode


    
    %% Angle-of-attack guidance
    switch guid.p.aoa_mode

        case 1 % Constant AOA
            
            cmd_aoa = guid.p.const_aoa; % Angle-of-attack command, rad
            
            
        case 2 % Velocity schedule
            
            % Requires monotonically increasing velocity domain
            len_aoa = guid.p.aoa_sched_len;
            cmd_aoa = lin_interp(guid.p.aoa_schedule(1:len_aoa,2),guid.p.aoa_schedule(1:len_aoa,1),calcs.mach);

            
        case 3 % Bank-AOA unified guidance (set above)
        
            % Do nothing, cmd_aoa set above
            
            
        otherwise % No guidance - issue constant AOA command

            cmd_aoa = guid.p.const_aoa; % Angle-of-attack command, rad           
        
            
    end % switch guid.p.aoa_mode
    
    
    
    %% Propulsive guidance
    switch guid.p.prop_mode
        
        case 1 % No propulsive guidance, i.e. do nothing
        
        case 2 % Terminal descent: gravity turn start logic
            
            % Assign inputs
            guid.gt.i.R = nav.s.r_pci; % Position vector (PCI), m
            guid.gt.i.V_pf = nav.s.v_pf_pci; % Planet-fixed velocity vector (PCI), m/s
                        
            % Check for ignition time
            guid.gt.s = guid_gt( guid.gt.i, guid.gt.s, guid.gt.p, guid.s.init_flag );
            
        case 3 % Placeholder for future propulsive guidance
            
        case 4 % Aerocapture: one- and two-stage jettison drag modulation

            % Assign inputs
            guid.sej_a.i.t = nav.s.t; % Time, s
            guid.sej_a.i.R_pci = nav.s.r_pci; % Position vector (PCI), m
            guid.sej_a.i.V_inrtl_pci = nav.s.v_inrtl_pci; % Inertial velocity vector (PCI), m/s
            guid.sej_a.i.V_pf_pci = nav.s.v_pf_pci; % Planet-fixed velocity vector (PCI), m/s
            guid.sej_a.i.A_sens_pci = nav.s.a_sens_pci; % Sensed acceleration vector (PCI), m/s^2 
            
            % Call guidance
            guid.sej_a.s = guid_sej_a( guid.sej_a.i, guid.sej_a.s, guid.sej_a.p, guid.s.init_flag );
            
            cmd_t_jettison = guid.sej_a.s.tj_curr; % s
            cmd_area_ref = guid.sej_a.s.cmd_area_ref; % m^2
            
        case 5 % Aerocapture: multi-stage jettison drag modulation

        case 6 % Aerocapture: continuously variable drag modulation
    
            % Assign inputs
            guid.dma_cv.i.t = nav.s.t; % Time, s
            guid.dma_cv.i.R_pci = nav.s.r_pci; % Position vector (PCI), m
            guid.dma_cv.i.V_inrtl_pci = nav.s.v_inrtl_pci; % Inertial velocity vector (PCI), m/s
            guid.dma_cv.i.V_pf_pci = nav.s.v_pf_pci; % Planet-fixed velocity vector (PCI), m/s
            guid.dma_cv.i.A_sens_pci = nav.s.a_sens_pci; % Sensed acceleration vector (PCI), m/s^2 
            
            % Call guidance
            guid.dma_cv.s = guid_dma_cv( guid.dma_cv.i, guid.dma_cv.s, guid.dma_cv.p, guid.s.init_flag );

            % Assign area command
            cmd_area_ref = guid.dma_cv.s.cmd_area_ref; % m^2
            
        case 7 % Entry: single-event jettison drag modulation (with parachute range trigger)

            % Assign inputs
            guid.sej_e.i.t = nav.s.t; % Time, s
            guid.sej_e.i.R_pci = nav.s.r_pci; % Position vector (PCI), m
            guid.sej_e.i.V_inrtl_pci = nav.s.v_inrtl_pci; % Inertial velocity vector (PCI), m/s
            guid.sej_e.i.V_pf_pci = nav.s.v_pf_pci; % Planet-fixed velocity vector (PCI), m/s
            guid.sej_e.i.A_sens_pci = nav.s.a_sens_pci; % Sensed acceleration vector (PCI), m/s^2

            % Call guidance
            guid.sej_e.s = guid_sej_e( guid.sej_e.i, guid.sej_e.s, guid.sej_e.p, guid.s.init_flag );
            
            % Assign commands
            cmd_t_jettison = guid.sej_e.s.t_j_curr; % s
            cmd_area_ref = guid.sej_e.p.area_ref(2); % m^2
            cmd_deploy_decel = guid.sej_e.s.deploy_decel; % nd
                        
        case 8 % Entry: multi-stage jettison drag modulation
        
        case 9 % Entry: continously variable jettison drag modulation

        otherwise % No propulsive guidance, i.e. do nothing

    end % switch guid.p.prop_mode

    
    %% Assign commands
    guid.cmd.bank = cmd_bank; % rad
    guid.cmd.aoa = cmd_aoa; % rad
   
    guid.cmd.t_jettison = cmd_t_jettison; % s    
    guid.cmd.area_ref = cmd_area_ref; % m^2
    
    guid.cmd.deploy_decel = cmd_deploy_decel; % nd
    
    %% Reset initialization flag if true (after first pass)
    if guid.s.init_flag
        guid.s.init_flag = logical(false);
    end

end % rate limiter

end % guidance()

