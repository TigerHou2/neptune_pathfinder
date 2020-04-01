% guid_sej_a.m 
%   Numeric predictor-corrector aerocapture guidance algorithm for
%       single-event jettison systems
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

function [s] = guid_sej_a( i, s, p, init_flag )
%#codegen

%% First-pass Initialization
if init_flag
    s = init(i, s, p);
end

%% Preliminary calculations
% Vector magnitudes
s.A_mag = norm(i.A_sens_pci); % m/s^2, sensed acceleration magnitude
s.V_pf_mag = norm(i.V_pf_pci); % m/s, planet-relative velocity magnitude

%% Sequencer
% Choose first step according to current phase
switch s.phase
    case 1
        s.next_step = uint8(1);
    case 2
        s.next_step = uint8(2);
    case 3
        s.next_step = uint8(3);
    case 4
        s.next_step = uint8(4);
    otherwise
        s.next_step = uint8(5);
end
% Sequence through steps
while (s.next_step ~= uint8(5))
    switch s.next_step
        case 1 % pre-entry hold
            s = pre_entry( i, s, p );
        case 2 % numeric predictor-corrector
            s = npc( i, s, p );
        case 3 % post-jettison hold
            s = post_jettison( i, s, p );
        case 4 % exit coast
            s = atm_exit( i, s, p );           
        otherwise % Shouldn't be possible
            s.next_step = uint8(5);
    end % switch s.next_step
end % while
   

end % guid_drag_jettison



function [ s ] = init(i, s, p)
% First-pass initialization function

%% Initialize states
s.jettison(1) = logical(false); % nd, jettison flag
s.jettison(2) = logical(false); % nd, jettison flags
s.init_ss = logical(false);
s.phase = uint8(1); % nd, current guidance phase
s.next_step = uint8(1); % nd, next guidnace phase to evaluate on this cycle
s.A_mag = double(0); % m/s^2, sensed acceleration magnitude
s.t_inc = p.t_inc_max; % s
s.tj_curr = p.tj_ini(1); % s
s.tj_next = p.tj_ini(1); % s
s.K_dens = 1; % nd, initially assume on-board model is correct
s.r_ap = 0; % m
s.delta_ap = 0; % m
s.trust_region = p.trust_region_ini;

if p.mode == uint8(2)
    s.cmd_area_ref = p.area_ref(3);
else
    s.cmd_area_ref = p.area_ref(2);
end

s.tj = [0;0];
s.ae = [0;0];

end % init



function [ s ] = pre_entry(i, s, p)
% Pre-entry hold prior to reaching sensible atmosphere

%% Test for atmospheric entry
% TEST: has the sensible atmosphere been reached yet?
if s.A_mag > p.A_sens_atm
    % YES, sensible atmosphere has been reached; proceed to NPC phase    
    s.phase = uint8(2); % NPC phase
    s.next_step = uint8(2); % run NPC
else
    % NO, sensible atmosphere has not been reached; do nothing    
    s.next_step = uint8(4); % Execution complete
end

end % pre_entry



function [ s ] = npc(i, s, p)
% Numeric predidctor-corrector targeting algorithm to determine when to
% jettison drag device

%% Check for jettison
if s.tj_curr <= i.t    
    
    %% Jettison complete, switch to exit phase
    
    s.jettison(1) = true; % set flag
    s.phase = uint8(3); % exit phase  
    s.next_step = uint8(5); % exit phase

elseif p.mode
    %% Execute predictor-corrector
    
    % Estimate density multiplier based on current acceleration
    s = est_K_dens(i, s, p);

    % Initialize predictor-corrector
    y0 = [i.R_pci; i.V_inrtl_pci]; % multi, initial state vector
    t0 = i.t; % s, initial time
    s.iter = uint8(0); % nd, iteration number
    s.bound = [0;0]; % nd, reset bounding storage
    s.tj = [0;0];
    s.ae = [0;0];
    cflag = 0; % nd, corrector flag   
    pred_mode = p.mode; % set predictor to single-stage or two-stage mode
    
    while ~cflag

        s.iter = s.iter + uint8(1); % Increment iteration counter
        [t, y, pflag] = guid_sej_a_pred(y0, t0, s.tj_next, p.tj_ini(2), s.K_dens, pred_mode, p); % run predictor to determine final state
        [s, cflag, cstatus] = guid_sej_a_corr(t, y, pflag, s, p); % run corrector to adjust jettison time
    
        if s.iter >= p.iter_max
            % ACTION: exit, max iterations exceeded
            cflag = 2; 
        end
        
        % Force CBE jettison time to be greater than currnet time
        if s.tj_next < i.t
            s.tj_next = i.t;
        end
        
        % Check for convergence
        if (s.bound(1)*s.bound(2))>0
            if (s.tj(2)-s.tj(1)) < 0.01
                cflag = 1;
            end
        end
        
    end % while cflag  

    % Assign command
    if (s.bound(1)*s.bound(2))>0
        s.tj_curr = s.tj_next;
    elseif s.bound(1) == 1
        s.tj_curr = s.tj(1);
    elseif s.bound(2) == 1
        s.tj_curr = s.tj(2);
    else
        s.tj_curr = s.tj_next;
    end
    
    % Check for jettison this time step
    if s.tj_curr <= i.t
        s.jettison(1) = true; % set flag
        s.phase = uint8(3); % exit phase  
        s.next_step = uint8(5); % second stage targeting
    else    
        s.next_step = uint8(5); % Execution complete
    end
    
else
    s.next_step = uint8(5); % Execution complete

end

end % npc


%% Second-stage jettison logic
function [ s ] = post_jettison(i, s, p)

if p.mode == uint8(2)

    if (s.tj_curr < i.t) && s.init_ss
        % IF it's time to jettison second stage
        
        s.jettison(2) = true; % set flag        
        s.phase = uint8(4);
        s.next_step = uint8(4);
        
        
    else
        % UPDATE second stage CBD jettison time
        
        if ~s.init_ss
            % Initalize second-stage targeting
%             s.jettison = logical(false); % nd, jettison flag
            s.tj_curr = p.tj_ini(2); % s
            s.tj_next = p.tj_ini(2); % s
            s.tj = [0;0];
            s.ae = [0;0];
            s.cmd_area_ref = p.area_ref(2);
            % Set flag
            s.init_ss = logical(true);
        end
        
        % Estimate density multiplier based on current acceleration
        s = est_K_dens(i, s, p);

        % Initialize predictor-corrector
        y0 = [i.R_pci; i.V_inrtl_pci]; % multi, initial state vector
        t0 = i.t; % s, initial time
        s.iter = uint8(0); % nd, iteration number
        s.bound = [0;0]; % nd, reset bounding storage
        s.tj = [0;0];
        s.ae = [0;0];
        cflag = 0; % nd, corrector flag
        pred_mode = uint8(3); % two-stage jettison, second stage

        while ~cflag

            s.iter = s.iter + uint8(1); % Increment iteration counter
            [t, y, pflag] = guid_sej_a_pred(y0, t0, s.tj_next, p.tj_ini(2), s.K_dens, pred_mode, p); % run predictor to determine final state
            [s, cflag, cstatus] = guid_sej_a_corr(t, y, pflag, s, p); % run corrector to adjust jettison time

            if s.iter >= p.iter_max
                % ACTION: exit, max iterations exceeded
                cflag = 2; 
            end

            % Force CBE jettison time to be greater than currnet time
            if s.tj_next < i.t
                s.tj_next = i.t;
            end

            % Check for convergence
            if (s.bound(1)*s.bound(2))>0
                if (s.tj(2)-s.tj(1)) < 0.01
                    cflag = 1;
                end
            end

        end % while cflag  

        % Assign command
        if (s.bound(1)*s.bound(2))>0
            s.tj_curr = s.tj_next;
        elseif s.bound(1) == 1
            s.tj_curr = s.tj(1);
        elseif s.bound(2) == 1
            s.tj_curr = s.tj(2);
        else
            s.tj_curr = s.tj_next;
        end

        s.next_step = uint8(5); % Execution complete        
        
    end

else
    
    % Go to atmospheric exit phase
    s.phase = uint8(4); % Atmospheric exit
    s.next_step = uint8(4); % Go to next phase

end
    
end % post_jettison


%% Post-drag-device-jettison hold, performs no actions
function [ s ] = atm_exit(i, s, p)

    % Do nothing
    s.next_step = uint8(5); % execution complete

end % post_jettison


%% Density factor estimator
function [ s ] = est_K_dens(i, s, p)

    if (p.mode == uint8(2)) && (s.phase == uint8(3))
        idx = uint8(3);
    else
        idx = uint8(1);
    end

    % Estimate density
    dens_est = (2*p.mass*s.A_mag) / (s.V_pf_mag*s.V_pf_mag*p.area_ref(idx)*p.cd); % kg/m^3
    alt = norm(i.R_pci) - p.p_r; % m
    dens_model = lin_interp(p.atm_table(:,1), p.atm_table(:,2),alt); % kg/m^3

    % Density factor
    K_dens = dens_est/dens_model; % nd

    % Limit to min/max values
    K_dens = median([p.K_dens_min, K_dens, p.K_dens_max]); % nd

    % Low-pass filter
    s.K_dens = p.K_dens_gain*K_dens + (1-p.K_dens_gain)*s.K_dens; % nd

end % est_K_dens
