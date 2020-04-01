% guid_sej_a_corr.m 
%   Corrector for single-event jettison aerocapture guidance algorithm
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
%   t - double(1), s, current time
%   y - double(6), multi, final state from predictor
%   pflag - int(1), nd, predictor termination flag
%   s - struct(1), multi, SEJ_A state structure
%   p - struct(1), multi, SEJ_A parameter structure
%   
% Output:
%   s - struct(1), multi, SEJ_A state structure
%   cflag - int(1), nd, corrector termination flag
%   c_status - int(1), nd, corrector status flag

function [s, cflag, cstatus] = guid_sej_a_corr(t,y,pflag,s,p)
%#codegen

cflag = 0;
cstatus = 0;


% %% Compute apoapse and apoapse error
% R = y(1:3); % m, position vector
% R_mag = norm(R); % m, position vector magnitude
% V_inrtl = y(4:6); % m/s, inertial velocity vector
% V_inrtl_mag = norm(V_inrtl); % m/s, inertial velocity vector magnitude
% 
% H = cross(R,V_inrtl); % m^2/s, specific angular momentum vector (BMW 1.4-3)
% E = cross(V_inrtl,H)/p.mu - R/R_mag; % nd, eccentricity vector (BMW 1.5-10)
% ecc = norm(E); % nd, eccentricity
% energy = V_inrtl_mag^2/2 - p.mu/R_mag; % m^2/s^2 (BMW 1.4-2)
% sma = -0.5 * p.mu/energy; % m, semi-major axis (BMW 1.6-3)
% s.r_ap = sma*(1+ecc); % m, apoapse radius (BMW 1.5-8)
% 
% s.delta_ap = s.r_ap - (p.tgt_ap + p.p_r); % m, apoapse error

%% Old corrector

% % Update jettison time
% s.tj_curr = s.tj_next; % set current jettison time to previous "next" jettison time
% 
% Correct
% % TEST: is apoapse error tolerance satisifed?
% if abs(s.delta_ap) < p.tol_ap 
%     % YES, apoapse error within tolerance
%     
%     % ACTION: store data and exit, solution found
%     if s.tj_curr ~= s.tj(2) % new data
%         s.ngood = median([0,s.ngood+1,2]);
%     else % update newest data only
%         s.ngood = median([0,s.ngood+1,1]);
%     end
%     s.tj(1) = s.tj(2);
%     s.tj(2) = s.tj_curr;
%     s.ae(1) = s.ae(2);
%     s.ae(2) = s.delta_ap;
% 
%     if s.ngood == 2
%     % ACTION: polate subject to trust region
%         s.tj_next = polate(s);
%         cflag = 0; % do not exit
% 
%     else
%         % ACTION: take tiny step towards target and rerun
%         s.tj_next = s.tj_curr + sign(s.delta_ap)*0.02;
%         cflag = 0; % do not exit
% 
%     end
%        
% else % Not within specified tolerance
%     
%     if s.r_ap < 0 % HYPERBOLIC EXIT CONDITION
%         % ACTION: take big step towards capture region
%         
%         s.tj_next = s.tj_curr + 3*s.t_inc;
%         cflag = 0; % run again
%         cstatus = 21;
%         
%     else % CAPTURE (ELLIPTIC EXIT CONDITION)
%     
%         if s.r_ap < (p.p_r + 100e3) % ENTRY (IMPACT)
%             
%             if s.ngood == 0
%                 % ACTION: take big step towards aerocapture region
%                 s.tj_next = s.tj_curr - 3*s.t_inc;
%                 cflag = 0; % run again
%                 
%             elseif s.ngood == 1
%                 % ACTION: go halfway to previous good guess
%                 s.tj_next = s.tj(2) - (s.tj(2)-s.tj_curr)/2;
%                 s.trust_region = s.trust_region / 2;
%                 cflag = 0; % run again
% 
%             else
%                 % ACTION: reduce size of trust region, adjust guess
%                 s.trust_region = s.trust_region / 2;
%                 s.tj_next = polate(s);
%                 cflag = 0; % run again
%                 
%             end
%             
%         else % AEROCAPTURE (ORBIT)
%                           
%             if s.tj_curr ~= s.tj(2) % new data
%                 s.ngood = median([0,s.ngood+1,2]);
%             else % update newest data only
%                 s.ngood = median([0,s.ngood+1,1]);
%             end
%             s.tj(1) = s.tj(2);
%             s.tj(2) = s.tj_curr;
%             s.ae(1) = s.ae(2);
%             s.ae(2) = s.delta_ap;
%             
%             if s.ngood == 2
%                 % ACTION: polate subject to trust region
%                 s.tj_next = polate(s);
%                 cflag = 0;
%                 
%             else
%                 % ACTION: step towards target and rerun
%                 s.tj_next = s.tj_curr + sign(s.delta_ap)*s.t_inc;
%                 cflag = 0;
%             
%             end
%             
%         end
%         
%     end
%     
% end


%% New, bisection-based corrector

% Update jettison time
% s.tj_curr = s.tj_next; % set current jettison time to previous "next" jettison time

% Some constants
big_step = 20; % s
medium_step = 10; % s

% Compute apoapse and apoapse error
R = y(1:3); % m, position vector
R_mag = norm(R); % m, position vector magnitude
V_inrtl = y(4:6); % m/s, inertial velocity vector
V_inrtl_mag = norm(V_inrtl); % m/s, inertial velocity vector magnitude
energy = V_inrtl_mag^2/2 - p.mu/R_mag; % m^2/s^2 (BMW 1.4-2)
sma = -0.5 * p.mu/energy; % m, semi-major axis (BMW 1.6-3)
H = cross(R,V_inrtl); % m^2/s, specific angular momentum vector (BMW 1.4-3)
E = cross(V_inrtl,H)/p.mu - R/R_mag; % nd, eccentricity vector (BMW 1.5-10)
ecc = norm(E); % nd, eccentricity
s.r_ap = sma*(1+ecc); % m, apoapse radius (BMW 1.5-8)
s.delta_ap = s.r_ap - (p.tgt_ap + p.p_r); % m, apoapse error


if (s.bound(1)*s.bound(2))

    if s.delta_ap > 0 % trajectory is high
        s.tj(1) = s.tj_next;
        s.ae(1) = s.delta_ap;
        path = 1;
    else
        s.tj(2) = s.tj_next;
        s.ae(2) = s.delta_ap;
        path = 2;
    end
    s.tj_next = (s.tj(2)-s.tj(1))/2 + s.tj(1);
    
elseif sma < 0 % hyperbolic
    
    s.tj_next = s.tj_next + big_step; % big positive step
    path = 3;
    
else % Capture
    
    % Compute apoapse error
    
    if s.delta_ap > 0 % predicted apoapse above target
        
        if (s.delta_ap < s.ae(1)) || (s.bound(1) == 0)
            s.tj(1) = s.tj_next;
            s.ae(1) = s.delta_ap;
            s.bound(1) = 1;
            
            if s.bound(2) == 1
                s.tj_next = (s.tj(2)-s.tj(1))/2 + s.tj(1);
            else                
                s.tj_next = s.tj_next + medium_step;
            end
            path = 4;
        else
            s.tj_next = s.tj_next + medium_step;
            path = 5;
        end
        
    else % predicted apoapse below target
        if (s.delta_ap > s.ae(2)) || (s.bound(2) == 0)
            s.tj(2) = s.tj_next;
            s.ae(2) = s.delta_ap;
            s.bound(2) = 1;
            if s.bound(1) == 1
                s.tj_next = (s.tj(2)-s.tj(1))/2 + s.tj(1);
            else
                s.tj_next = s.tj_next - medium_step;
            end
            path = 6;
        else        
            s.tj_next = s.tj_next - medium_step;
            path = 7;
        end
        
    end
    
end

% fprintf('\ni=%2d,tj=[%f,%f],ae=[%f,%f],tjn=%f,bd=[%1d,%1d]',s.iter, ...
%     s.tj(1),s.tj(2),s.ae(1),s.ae(2),s.tj_next,s.bound(1),s.bound(2) );


end % guid_sej_a_corr()


function t_new = polate(s)

    d = (s.ae(2)-s.ae(1));
    f = (s.tj(2)-s.tj(1))/d;
    t_new = s.tj(1) - f*s.ae(1); % s
    t_new = median([s.tj(2)-s.trust_region, t_new, s.tj(2)+s.trust_region]);

end