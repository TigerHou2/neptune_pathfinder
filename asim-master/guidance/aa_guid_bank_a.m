% aa_guid_bank_a.m 
%   Second-order control law bank angle modulation for aerocapture
%
% Author:   Tiger Hou
% Created:  02/18/2020
% Modified: 02/29/2020
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
%
% Sources:
%   - [1] Mars Aerocapture Using bank Modulation

function [s] = aa_guid_bank_a( i, s, p, init_flag )
%#codegen

%% Coordinate Transformation
% s.r_pf = i.r_pci;
% s.v_pf = i.v_pci;% - cross(p.omega,i.r_pci);

%% First-pass Initialization
if init_flag
    s = init(i, s, p);
end

%% Preliminary calculations

% planet-relative flight path angle
s.fpa  = pi/2-atan2(norm(cross(i.r_pci,i.v_pci)),dot(i.r_pci,i.v_pci));

%================
s = update_yb(i,s,p);
% [1] eqn.48
if s.yb==Inf
    s.k = 0;
else
    s.k = log(s.yb)/(s.yb-1);
end
%================

% non-dimensional speed parameter, [1] eqn.12
% s.x    = 2*log(p.Ve/norm(i.v_pci));
s.x    = (log(p.Ve/norm(i.v_pci)))^2;

% [1] eqn.53
s.A = 1/sqrt(p.beta*p.R)*(1+2*s.k*(1-p.delta)/p.eps/s.E/sqrt(p.beta*p.R));

% [1] eqn.54
temp = cos(p.fpa_e) ...
            -(p.eps*s.E/2/sqrt(p.beta*p.R) ...
                +(1+s.k)*(1-p.delta)/(p.beta*p.R) ...
                +2*s.k*(1-p.delta)^2/p.eps/s.E/(p.beta*p.R)^1.5);
s.fpa_0 = acos(max(min(temp,1),-1));

% [1] eqn.55
temp = (tan(s.fpa_0/2)+tan(s.fpa/2))*(tan(s.fpa_0/2)-tan(p.fpa_e/2)) ...
         /(tan(s.fpa_0/2)-tan(s.fpa/2))/(tan(s.fpa_0/2)+tan(p.fpa_e/2));
s.s = s.A/sin(s.fpa_0)*log(abs(temp));
        %((temp-1)-1/2*(temp-1)^2+1/3*(temp-1)^3-1/4*(temp-1)^4); % 4th order Taylor expansion of ln(temp)

% Test
% s.s = s.s + sqrt(p.beta/p.R)*norm(i.v_pci);
        
s = update_yb(i,s,p);
s = update_rho(i,s,p);

% [1] eqn.11
s.y = s.rho/p.rho_e;

s = update_sf(i,s,p);

% [1] eqn.74 (second order control)
s.E = (2*(p.fpa_f-s.fpa)-2*(1-p.delta)/sqrt(p.beta*p.R)*(s.s_f-s.s)) ...
     /(p.x_f-s.x-2*p.delta/(p.beta*p.R)*log(abs(s.y)));

% Test
% I = - p.delta/(p.beta*p.R)*log(abs(s.y))^2 ...
%     - 2*(1-p.delta)/s.E/sqrt(p.beta*p.R)...
%         *(log(abs(s.y))+s.k+2*s.k*(1-p.delta)/p.eps/s.E/sqrt(p.beta*p.R))*s.s ...
%     + 2*(s.fpa-p.fpa_e)/s.E...
%         *(2*s.k*(1-p.delta)/p.eps/s.E/sqrt(p.beta*p.R)+log(abs(s.y))...
%             -2*s.k*p.beta*p.R*(1-cos(p.fpa_e))...
%                 /(2*s.k*(1-p.delta)+p.eps*s.E*sqrt(p.beta*p.R)));
% I = 1;
% s.E = (2*(p.fpa_f-s.fpa)-2*(1-p.delta)/sqrt(p.beta*p.R)*(s.s_f-s.s)) ...
%      /(p.x_f-s.x+2*p.delta/(p.beta*p.R)*log(abs(s.y)));
% s.E = (2*(s.fpa-p.fpa_e)-2*(1-p.delta)/sqrt(p.beta*p.R)*(s.s)) ...
%      /(s.x+2*p.delta/(p.beta*p.R)*(log(abs(s.y))+I));

% [1] eqn.72 (first order control law)
% s.E = 4*tan(abs(p.fpa_f-s.fpa)/2)/(p.x_f-s.x);

% E_temp = s.E;

% constrain E to possible values
s.E = sign(s.E)*min(abs(s.E),p.cl/p.cd);
% [1] eqn.25
s.cmd_bank = real(acos(real(s.E)/p.cl*p.cd));

if s.fpa < deg2rad(-1.5) && ~any(isnan(i.v_pci))
    s.E = p.cl/p.cd;
    s.cmd_bank = 0;
end


% test values
% cmd_bank = s.cmd_bank;
% yb = s.yb;
% E = s.E;
% k = s.k;
% x = s.x;
% y = s.y;
% A = s.A;
% s_f = s.s_f;
% S = s.s;
% fpa_0 = s.fpa_0;
% fpa = s.fpa;
% A = A;
   

end % aa_guid_bank_a



function [ s ] = init(i, s, p)
% First-pass initialization function

%% Coordinate Transformation
% s.r_pf = i.r_pci;
% s.v_pf = i.v_pci;% - cross(p.omega,i.r_pci);

%% Initialize states

% [1] eqn.25
s.E = -p.cl/p.cd; % initialize assuming bank angle = pi
s.E = pi;

s = update_yb(i,s,p);

% [1] eqn.48
if s.yb==Inf
    s.k = 0;
else
    s.k = log(s.yb)/(s.yb-1);
end

s = update_rho(i,s,p);
p.rho_e = s.rho;

% [1] eqn.11
s.y = s.rho/p.rho_e;

s = update_sf(i,s,p);

% non-dimensional speed parameter, [1] eqn.12
s.x = (log(p.Ve/norm(i.v_pci)))^2;

% [1] eqn.53
s.A = 1/sqrt(p.beta*p.R)*(1+2*s.k*(1-p.delta)/p.eps/s.E/sqrt(p.beta*p.R));

% [1] eqn.54
temp = cos(p.fpa_e) ...
            -(p.eps*s.E/2/sqrt(p.beta*p.R) ...
                +(1+s.k)*(1-p.delta)/(p.beta*p.R) ...
                +2*s.k*(1-p.delta)^2/p.eps/s.E/(p.beta*p.R)^1.5);
s.fpa_0 = acos(max(min(real(temp),1),-1));

% [1] eqn.55
temp = (tan(s.fpa_0/2)+tan(s.fpa/2))*(tan(s.fpa_0/2)-tan(p.fpa_e/2)) ...
         /(tan(s.fpa_0/2)-tan(s.fpa/2))/(tan(s.fpa_0/2)+tan(p.fpa_e/2));
s.s = s.A/sin(s.fpa_0)*log(abs(temp));
        %((temp-1)-1/2*(temp-1)^2+1/3*(temp-1)^3-1/4*(temp-1)^4); % 4th order Taylor expansion of ln(temp)

        
end % init

function [s] = update_yb(i,s,p)
% reevaluates the minimum of the non-dimentional altitude parameter

F = @(y) p.eps*s.E/2*(y-1)+(1-p.delta)/sqrt(p.beta*p.R)*log(abs(y)) - ...
    sqrt(p.beta*p.R)*(cos(0)-cos(p.fpa_e));
y0 = 1;
F_eval = F(y0);
while abs(F_eval) > 1e-5
    y0 = y0 - sign(F_eval)*y0*0.4;
    F_eval = F(y0);
end
s.yb = y0;

end

function [s] = update_rho(i,s,p)
% reevaluates the non-dimentional altitude parameter

s.rho = (2*p.mass*norm(i.a_pci))/(norm(i.v_pci*1000)^2*p.area_ref*p.cd); % kg/m^3

end

function [s] = update_sf(i,s,p)
% reevaluates the non-dimentional arc length at atmosphere exit

% [1] eqn.55
s.s_f = s.A/sin(s.fpa_0)* ... 
        log((tan(s.fpa_0/2)+tan(p.fpa_f/2))*(tan(s.fpa_0/2)-tan(p.fpa_e/2)) ...
           /(tan(s.fpa_0/2)-tan(p.fpa_f/2))/(tan(s.fpa_0/2)+tan(p.fpa_e/2)));

end

