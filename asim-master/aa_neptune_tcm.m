%% neptune_tcm.m
%
% Author: Tiger Hou
%
% Description:
%   Simulates a s/c approaching Neptune on a hyperbolic orbit.
%   The s/c makes two TCM maneuvers:
%       TCM-1: slows down s/c while maintaining same EFPA
%       TCM-2: adjusts EFPA to a desired value.
%
%
%% Initialization

close all hidden
clear;clc

setup
neptune = get_planetary_data(8,2);

fpa_cur = deg2rad(-8);  % current efpa
fpa_tgt = deg2rad(-12); %  target efpa

r_ei = [25264000; 0; 0];  % EI position

x_axis = r_ei / norm(r_ei);         % calculate the s/c coordinate frame
z_axis = [0;0;1];                   % so velocity is always perpendicular
y_axis = cross(z_axis,x_axis);      % to the position vector, and also
rotn = [x_axis, y_axis, z_axis];    % parallel to the equator

v_ei = 30.5e3 * rotn * [sin(fpa_cur); cos(fpa_cur); 0]; % EI velocity

tcm_1_leadtime = 14;    % days
tcm_2_leadtime = 1;     % days

visualize = false;

if visualize

    disp_orbit(r_ei,v_ei,neptune.mu,30,0.02,0,'red')

    hold on
    [x,y,z] = sphere;
    x = x*neptune.r_m;
    y = y*neptune.r_m;
    z = z*neptune.r_m;
    nep_disp = surf(x,y,z);
    alpha(nep_disp,0.5)
    hold off

    view([1,1,1])
    
end


%% TCM-1: delay s/c delivery

bp_time = - tcm_1_leadtime - tcm_2_leadtime;

% backpropagate until the moment that pathfinder enters Neptune
[r1,v1] = TimeProp_V3(r_ei,v_ei,neptune.mu,bp_time);

% get current orbit parameters
[a1,e1,i1,omg1,w1,f1] = Get_Orb_Params(r1,v1,neptune.mu);

% optimize TCM-1 (x = [a,e,f])
fun = @(x) abs(x(3)-a1);
x0 = [a1;norm(e1);0];
lb = [-Inf,0,-2*asin(1/norm(e1))];
ub = [0,Inf,2*asin(1/norm(e1))];
options = optimoptions(@fmincon,'display','iter');
x = fmincon(fun,x0,[],[],[],[],lb,ub,@(x)nonlcon(x,norm(r_ei),fpa_tgt),...
            options);
a1_tcm = x(1);
e1_tcm = x(2);
f_ei_1_tcm = x(3);
r1_tcm = r1;

v1_tcm_norm = sqrt(neptune.mu*(2/norm(r1_tcm)-1/a1_tcm));

f1_tcm = acos((a1_tcm*(e1_tcm^2-1)/norm(r1_tcm)-1)/e1_tcm);
f1_tcm = -sign(f1_tcm)*f1_tcm;

% find new velocity vector
fpa1 = atan(norm(e1)*sin(f1)/(1+norm(e1)*cos(f1)));
fpa1_tcm = atan(e1_tcm*sin(f1_tcm)/(1+e1_tcm*cos(f1_tcm)));

x_axis = r1_tcm / norm(r1_tcm);     % calculate the s/c coordinate frame
z_axis = [0;0;1];
y_axis = cross(z_axis,x_axis);
rotn = [x_axis, y_axis, z_axis];

v1_tcm = v1_tcm_norm * rotn * [sin(fpa1_tcm); cos(fpa1_tcm); 0];

dv1 = norm( v1 - v1_tcm );


% visualize
% disp_orbit(r1_tcm,v1_tcm,neptune.mu,100,105,105,'red')
% 
% hold on
% [x,y,z] = sphere;
% x = x*neptune.r_m;
% y = y*neptune.r_m;
% z = z*neptune.r_m;
% nep_disp = surf(x,y,z);
% alpha(nep_disp,0.5)
% hold off
% 
% view([1,1,1])


%% TCM-2: adjust s/c efpa





%% Nonlinear Constraint Functions

function [c,ceq] = nonlcon(x,r,fpa)

    c(1) = x(1);
    c(2) = -(x(2)-1);
    ceq(1) = r - ( x(1) * (x(2)^2-1) ) / ( 1 + x(2)*cos(x(3)) );
    ceq(2) = tan(fpa) - x(2)*sin(x(3)) / ( 1 + x(2)*cos(x(3)) );
    ceq(3) = sign(fpa) - sign(x(3));

end