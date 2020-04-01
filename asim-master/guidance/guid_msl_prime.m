% guid_msl_prime.m 
%   MSL' guidance scheme (loosely based on MSL) - METRIC UNITS
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
%   gi - struct(1), multi, guidance input structure
%   gs - struct(1), multi, guidance state structure
%   gp - struct(1), multi, guidance parameter structure
%   init_flag - logical(1), nd, first-pass initialization flag
%   
% Output:
%   gs - struct(1), multi, guidance state structure

function [ gs ] = guid_msl_prime( gi, gs, gp, init_flag ) %#codegen


%% First-pass initialization
if init_flag
    gs = guid_init( gi, gs, gp );
end


%% Perform targeting tasks
gs = target( gi, gs, gp );


%% Sequence through proper functions
while (gs.next_step ~= uint8(5))

    switch gs.next_step
        case 1 % pre_bank
            gs = pre_bank( gi, gs, gp );
            
        case 2 % final
            gs = final( gi, gs, gp );
       
        case 3 % heading_alignment
            gs = heading_alignment( gi, gs, gp );
       
        case 4 % lateral_logic
            gs = lateral_logic( gi, gs, gp );
       
        otherwise % Shouldn't be possible
            gs.next_step = uint8(5);
    
    end % switch gs.next_step   

end % while

end % guid_msl_prime


%% Guidance Initialization
function [ gs ] = guid_init( gi, gs, gp )

ds = double(0);
dv3 = double(zeros(3,1));

% Initialize temporary parameters
ECEF_to_ECI   = double([0,0,0; 0,0,0; 0,0,0]);
U_E_pole_ECEF = dv3;
R_target_ECEF = dv3;
R_target_ECI  = dv3;
temp_s1       = ds;
temp_s2       = ds;
temp_s3       = ds;


%% Compute targeting vectors
% Earth spin axis in ECEF
U_E_pole_ECEF = double([0; 0; 1]); % nd

% Compute initial target position vector in ECEF
temp_s1 = (1.0 - gp.E_flatness)^double(2); % nd
temp_s2 = gp.E_radius_eq / sqrt( (cos(gp.tgt_geod_lat))^double(2) + ...
	temp_s1 * (sin(gp.tgt_geod_lat))^double(2) ); % m
temp_s3 = (temp_s2 + gp.tgt_geod_alt)*cos(gp.tgt_geod_lat); % m
R_target_ECEF(1) = temp_s3 * cos(gp.tgt_lon); % m
R_target_ECEF(2) = temp_s3 * sin(gp.tgt_lon); % m
R_target_ECEF(3) = (temp_s1*temp_s2 + gp.tgt_geod_alt) * ...
	sin(gp.tgt_geod_lat); % m

% Transform Earth spin axis and target position vector to ECI
ECEF_to_ECI = eye(3);
gs.U_E_pole = ECEF_to_ECI * U_E_pole_ECEF; % nd
R_target_ECI = ECEF_to_ECI * R_target_ECEF; % m

% Initialize targeting vectors
gs.U_tgt_init = R_target_ECI/norm(R_target_ECI,2); % nd
gs.U_tgt_east = cross( gs.U_E_pole, gs.U_tgt_init ); % nd
gs.U_tgt_eq   = cross( gs.U_tgt_east, gs.U_E_pole ); % nd

%% Initialize guidance states
% Flags
gs.tgt_overflown        = logical(false); % target not reached, nd
gs.V_rel_switch_reached = logical(false); % use inertial velocity, nd

% Integer counters
gs.phase          = uint8(1); % start with Initial Roll, nd
gs.next_step      = uint8(1);

% Other states
gs.alt_rate         = ds; % altitude rate, m/s
gs.ang_to_tgt       = ds; % range angle to target, rad
gs.cmd_bank         = gp.pre_bank; % set to initial command, rad
gs.cmd_bank_sign    = int8(1); % bank direction, nd
gs.cmd_lodv         = cos(gp.pre_bank)*gp.lod_nom; % set initial command to 15 deg, nd
gs.D_mag            = ds; % drag acceleration magnitude, m/s^2
gs.lat_ang          = ds; % lateral angle, rad
gs.lat_db           = ds; % lateral deadband, rad
gs.lod_work         = gp.lod_nom; % assume nominal lift-to-drag ratio, nd
gs.lodv_lim_work    = gp.lodv_lim; % assume nominal lift-to-drag ratio, nd
gs.range_final      = ds; % predicted Final phase range, m
gs.range_to_tgt     = ds; % range to target, m
gs.ref_alt_rate     = ds; % gs.rdotref, double, m/s
gs.ref_D_mag        = ds; % gs.drefr, double, m/s^2
gs.ref_f1           = ds; % gs.f1, double, m/m/s^2
gs.ref_f2           = ds; % gs.f2, double, m/m/s
gs.ref_f3           = ds; % gs.f3, double, m
gs.ref_range        = ds; % gs.rtogo, double, m
gs.tgt_transfer_ang = ds; % outputs.wt, double, rad
gs.time             = ds; % current guidance time, s
gs.time_init        = gi.timetag; % log guidance start time, s
gs.U_norm_traj      = dv3; % outputs.UNIbar, double(3), nd
gs.U_tgt            = gs.U_tgt_init; % outputs.URTbar, double(3), nd
gs.V_mag            = ds; % outputs.vData, double, m/s
gs.V_norm_sq        = ds; % outputs.vsq, double, nd
gs.V_work           = gi.V_ocg_wrt_Eartho_icrf_ICRF; % outputs.Vbar, double(3), m/s

% Call target twice to set lateral angle
gs = target( gi, gs, gp );
gs = target( gi, gs, gp );

% Initialize bank command to proper sign
if gs.lat_ang ~= double(0)
  gs.cmd_bank_sign = int8(-sign(gs.lat_ang)); % nd
else
  gs.cmd_bank_sign = int8(1); % nd
end

end % guid_init()



%% Apollo targeting function
function [ gs ] = target( gi, gs, gp )

% Compute guidance time
gs.time = gi.timetag - gs.time_init; % s

% TEST: has the relative velocity flag been set previously?
% if gs.V_rel_switch_reached == false
%   % NO, use inertial velocity
%   gs.V_work = gi.V_ocg_wrt_Eartho_icrf_ICRF; % m/s
%   
% else  
  % YES, use planet-relative velocity 
  gs.V_work = gi.V_ocg_wrt_Eartho_itrf_ICRF; % m/s
%   fprintf('switching to relative velocity @ t = %f, V = %f\n',gs.time,norm(gs.V_work,2))
  
% end


%% Compute vector magnitudes and quantities
% Velocity magnitude
gs.V_mag = norm(gs.V_work,2); % m/s
% Square of velocity magnitude, normalized by satellite velocity
gs.V_norm_sq = (gs.V_mag/gp.V_sat)^double(2); % nd
% Unit vector along current position vector
temp_v1 = gi.R_ocg_wrt_Eartho_ICRF / norm(gi.R_ocg_wrt_Eartho_ICRF,2); % nd
% Current altitude rate
gs.alt_rate = dot(gs.V_work, temp_v1); % m/s
% Vector normal to current trajectory plane
temp_v2 = cross(gs.V_work, temp_v1); % m/s
% Unit normal to current trajectory plane
gs.U_norm_traj = temp_v2 / norm(temp_v2,2); % nd
% Current acceleration magnitude, assumed to be aerodynamic only
gs.D_mag = norm(gi.Asens_onb_wrt_Eartho_icrf_OB,2); % m/s^2


%% Estimate target transfer angle
% TEST: has the relative velocity flag been set previously?
if gs.V_rel_switch_reached == false
  % NO, use inertial velocity

  % TEST: has the Final phase been reached?
%   if gs.phase < uint8(2)
%     % NO, Final phase not reached yet, 
%     % Use pre-Final phase method for target transfer angle calculation 
%     % (assumes a constant velocity flown to target)
%     gs.tgt_transfer_ang = gp.E_rotation_rate * ...
%       ( gp.tof_prm * gs.ang_to_tgt + gs.time ); % rad
% 
%   else   
    % YES, Final phase reached
    % Use inertial Final phase method for target transfer angle calculation 
    % (assumes current velocity flown all the way to target)
    gs.tgt_transfer_ang = gp.E_rotation_rate * ...
      ( gp.E_radius_eq * gs.ang_to_tgt / gs.V_mag + ...
      gs.time ); % rad

    % TEST: is velocity mag. low enough to switch to relative velocity?
    if (gs.V_mag - gp.V_rel_switch) <= double(0)
      % YES, switch to relative velocity
      gs.V_rel_switch_reached = true; % nd

    end
      % OTHERWISE, do not switch to relative velocity
      
%   end

else
  % YES, use relative velocity method for target transfer angle calculation (chase target)
  gs.tgt_transfer_ang = gp.E_rotation_rate * gs.time; % rad

end


%% Compute targeting angles and vectors
% Compute current target position unit vector
temp_s1 = cos( gs.tgt_transfer_ang ) - double(1); % nd
temp_s2 = sin( gs.tgt_transfer_ang ); % nd
gs.U_tgt = gs.U_tgt_init + gs.U_tgt_eq*temp_s1 + ...
	gs.U_tgt_east*temp_s2; % nd

% Ccompute lateral angle
gs.lat_ang = -dot( gs.U_tgt, gs.U_norm_traj ); % rad

% Compute longitudinal angle
gs.ang_to_tgt = acos( dot( gs.U_tgt, temp_v1 ) ); % rad

% Compute longitudinal range to target
gs.range_to_tgt = gs.ang_to_tgt * gp.E_radius_avg; % m

% determine next function call
switch gs.phase
    case 1 % Pre-bank Phase
        gs.next_step = uint8(1);
    case 2 % Final Phase
        gs.next_step = uint8(2);
    case 3 % Heading Alignment Phase
        gs.next_step = uint8(3);
    otherwise
        gs.next_step = uint8(5); % terminate guidance
end

end % target()



%% Pre-bank function
function [ gs ] = pre_bank( gi, gs, gp )

    if gs.D_mag > gp.D_sensible_atm        
        gs.phase = uint8(2); % Final Phase
        gs.next_step = uint8(2); % final()
    else        
        gs.cmd_lodv = gs.lod_work * cos(gp.pre_bank);
        gs.next_step = uint8(4); % lateral_logic() 
    end

end % pre_bank()



%% Apollo Final Phase
function [ gs ] = final( gi, gs, gp )

% TEST: is velocity greater than heading alignment transition velocity?
if gs.V_mag > gp.V_ha
    
    % TEST: has the vehicle previously overflown the target?
    if gs.tgt_overflown == false
        % NO, target has not been overflown       
        temp_v1 = cross( gs.U_tgt, gi.R_ocg_wrt_Eartho_ICRF ); % m

        % TEST: has target been overflown this cycle?
        if dot( temp_v1, gs.U_norm_traj ) > 0
            % NO, target has not been overflown this cycle                      
            % perform table look up
            y = lin_interp( gp.ref_trajectory(:,1), gp.ref_trajectory(:,2:7), gs.V_mag );
            
            % assign table lookup values
            gs.ref_alt_rate = y(1); % reference altitude rate, m/s
            gs.ref_D_mag    = y(2); % reference drag acceleration, m/s^2
            gs.ref_f2       = y(3); % partial of range-to-target WRT altitude rate, m/m/s
            gs.ref_f1       = y(4); % partial of range-to-target WRT drag acceleration, m/m/s^2
            gs.ref_range    = y(5); % reference range-to-target, m
            gs.ref_f3       = y(6); % partial of range-to-target WRT vertical L/D, m
            
            % predict range to target
            gs.range_final = gs.ref_range + ...
              gs.ref_f2*(gs.alt_rate - gs.ref_alt_rate) +...
              gs.ref_f1*(gs.D_mag - gs.ref_D_mag); % m

            % compute command
            % Assign single reference lodv based on reference relative velocity
            if(gp.ref_vel(end)~=0)  % Default reference velocity is zeros
                gp.ref_lodv = lin_interp( gp.ref_vel, gp.ref_lodv_vec, gs.V_mag);
%                 for pp = 1:(length(gp.ref_vel)-1)
%                     if((gs.V_mag > gp.ref_vel(pp,1)) && (gs.V_mag <= gp.ref_vel(pp+1,1)))
%                         gp.ref_lodv = lin_interp( gp.ref_vel, gp.ref_lodv_vec, V_mag);
%                         break;
%                     elseif(gs.V_mag > gp.ref_vel(length(gp.ref_vel),1))
%                         gp.ref_lodv = gp.ref_lodv_vec(length(gp.ref_vel),1);
%                         break;
%                     end
%                 end
%                 fprintf('%f %f\n',gs.V_mag,gp.ref_lodv)
            else
                gp.ref_lodv = gp.ref_lodv_vec(1);
            end
            
            gs.cmd_lodv = gp.ref_lodv + gp.overcontrol * ...
              (gs.range_to_tgt - gs.range_final) / gs.ref_f3; % nd
          
        else
            % YES, target has been overflown this cycle
            % set tgt_overflown flag
            gs.tgt_overflown = true; % nd

            % command lift down            
            gs.cmd_lodv = -gs.lod_work; % nd
        end
        
    else
        % YES, the target has been overflow on a previous cycle
        % command lift down        
        gs.cmd_lodv = -gs.lod_work;
        
    end

%     % continue to G-limiter (not implemented)
%     gs.phase = 5;
%     gs.next_step = 13;
    
    % Continue to lateral logic
    gs.next_step = uint8(4); 

else
    % transition to heading alignment phase
    gs.phase = uint8(3);
    gs.next_step = uint8(3); 
end    

end % final()



%% Heading alignment
function [ gs ] = heading_alignment( gi, gs, gp )

    %gs.cmd_lodv = 0.0; 
    k4 = 100;  % Proportional controller for the azimuth error
    gs.cmd_bank = -atan2(gs.lat_ang,gs.ang_to_tgt) * k4;
    % Limit bank angle to +/- 30deg
    if(abs(gs.cmd_bank) > (pi/6))
        gs.cmd_bank = sign(gs.cmd_bank) * pi/6;
    end
    
    gs.next_step = uint8(5); % Guidance execution complete
    
end



%% Lateral Logic
function [ gs ] = lateral_logic( gi, gs, gp)

% TEST: has the target been overflown?
if gs.tgt_overflown == logical(false)
  % NO, target has not been overflown 
  % Calculate lateral deadband
  gs.lat_db = gs.lod_work/gp.lat_db_gain * ...
    gs.V_norm_sq + gp.lat_db_min / gp.E_radius_avg; % rad

    if gs.V_mag > 3000
      gs.lat_db = gs.lat_db/2;
    end

  % TEST: bank revesals enabled?
  if (gs.D_mag > gp.D_sensible_atm) 
    % YES, bank reversals enabled--execute lateral logic
    
    % TEST: is the lift vector within 15 deg of max lift?
    if ( abs(gs.cmd_lodv) - gs.lodv_lim_work ) < double(0)
      % NO, the lift vector is not within 15 deg of max lift

      % TEST: does the heading error exceed the deadband?
      if ( double(gs.cmd_bank_sign)*gs.lat_ang - gs.lat_db ) > double(0)
        % YES, the heading error exceeds the deadband
        % Change bank direction
        gs.cmd_bank_sign = -gs.cmd_bank_sign; % nd      

      end
        % OTHERWISE, the heading error is acceptable, do nothing

    else
      % YES, the lift vector is within 15 deg of max lift
      % halve lateral deadbands to compensate for reduced capability
      gs.lat_db = gs.lat_db / double(2); % rad
      
      % TEST: is the lift vector in the same direction as the target?
      if ( double(gs.cmd_bank_sign) * gs.lat_ang ) > double(0)
        % NO, the vectors are not in the same direction

        % TEST: does the heading error exceed the deadband?
        if ( double(gs.cmd_bank_sign) * gs.lat_ang - gs.lat_db ) > double(0)
          % YES, the heading error exceeds the deadband
          % Change bank direction
          gs.cmd_bank_sign = -gs.cmd_bank_sign; % nd
          
        end
          % OTHERWISE, the heading error is acceptable, do nothing

      else
        % YES, the vectors are in the same direction
        
        gs.cmd_lodv = gs.lodv_lim_work * sign( gs.cmd_lodv ); % nd

      end

    end
    
  end
  % OTHERWISE, bank reversals prohibited. Skip lateral logic and assign bank command
  
end
  % OTHERWISE, target has been overflown
  
% TEST: is the commanded magnitude of L/D greater than vehicle capability?
if ( abs( gs.cmd_lodv/gs.lod_work ) - double(1) ) > double(0)
  % YES, the commanded magnitude is too high  
  % Reduce commanded magnitude to max allowable, maintaining proper sign
  gs.cmd_lodv = sign(gs.cmd_lodv) * gs.lod_work; % nd

end
  % OTHERWISE, the commanded magnitude is not too high, no need to change command

% Calculate appropriate bank angle, based on desired vertical L/D
gs.cmd_bank = double(gs.cmd_bank_sign) * ...
	acos( gs.cmd_lodv / gs.lod_work ); % rad

% Guidance execution complete
gs.next_step = uint8(5);

end