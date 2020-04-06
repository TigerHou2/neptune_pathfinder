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
mu = neptune.mu;

fpa_cur = deg2rad(-8);  % current efpa
fpa_tgt = deg2rad(-12); %  target efpa

r_ei_0 = [25264000; 0; 0];  % EI position

x_axis = r_ei_0 / norm(r_ei_0);     % calculate the s/c coordinate frame
z_axis = [0;0;1];                   % so velocity is always perpendicular
y_axis = cross(z_axis,x_axis);      % to the position vector, and also
rotn = [x_axis, y_axis, z_axis];    % parallel to the equator

v_ei_0 = 30.5e3 * rotn * [sin(fpa_cur); cos(fpa_cur); 0]; % EI velocity

leadtime_tcm1 = 210; % days, main s/c slowdown maneuver before original EI
leadtime_tcm2 = 1;  % days, main s/c FPA correction maneuver before modified EI
                    %       (this is also the main s/c delay time)

visualize = true;


%% TCM-1: delay s/c delivery

bp_time = - leadtime_tcm1;
dt = leadtime_tcm1 + leadtime_tcm2;

% backpropagate from pathfinder/main EI to TCM-1 time
[r1,v1] = TimeProp_V3(r_ei_0,v_ei_0,mu,bp_time);

% get current orbit parameters
[a1,e1,~,~,~,f1] = Get_Orb_Params(r1,v1,mu);

% get EI orbit parameters pre-TCM-1
[a_ei_0,e_ei_0,~,~,~,f_ei_0] = Get_Orb_Params(r_ei_0,v_ei_0,mu);

% optimize TCM-1 (x = [a,e,f])
ft =   @(x) 2*pi - acos( ( x(1)*(1-x(2)^2)-norm(r1) ) / x(2) / norm(r1) );
dfpa = @(x) abs( atan(   e1*sin( f1    ) / (1+  e1*cos( f1    )) ) - ...
                 atan( x(2)*sin( ft(x) ) / (1+x(2)*cos( ft(x) )) ) );
vt =   @(x) sqrt( mu * (2/norm(r1) - 1/x(1)) );
fun =  @(x) ( cos(dfpa(x)) * vt(x) - norm(v1) )^2 + ...
            ( sin(dfpa(x)) * vt(x) )^2;

x0 = [a_ei_0;norm(e_ei_0);f_ei_0];
lb = [-Inf,   1,   pi];
ub = [   0, Inf, 2*pi];
options = optimoptions(@fmincon,'display','none',...
                                'MaxFunctionEvaluations',20000,...
                                'MaxIterations',2500);
[x,~,exitflag] = fmincon(fun,x0,[],[],[],[],lb,ub,...
                      @(x)nonlcon_tcm1(x,r_ei_0,r1,fpa_cur,dt,mu),options);
a1_tcm = x(1);
e1_tcm = x(2);
f_ei_1_tcm = x(3);
r1_tcm = r1;

v1_tcm_norm = sqrt(mu*(2/norm(r1_tcm)-1/a1_tcm));

f1_tcm = acos((a1_tcm*(1-e1_tcm^2)/norm(r1_tcm)-1)/e1_tcm);
f1_tcm = 2*pi-sign(f1_tcm)*f1_tcm;

% find new velocity vector
fpa1 = atan(norm(e1)*sin(f1)/(1+norm(e1)*cos(f1)));
fpa1_tcm = atan(e1_tcm*sin(f1_tcm)/(1+e1_tcm*cos(f1_tcm)));

x_axis = r1_tcm / norm(r1_tcm);     % calculate the s/c coordinate frame
z_axis = [0;0;1];
y_axis = cross(z_axis,x_axis);
rotn = [x_axis, y_axis, z_axis];

v1_tcm = v1_tcm_norm * rotn * [sin(fpa1_tcm); cos(fpa1_tcm); 0];

dv1 = norm( v1 - v1_tcm );

disp(['TCM-1 Delta-v: ' num2str(dv1) ' m/s, Exit Flag: ' num2str(exitflag)])


%% TCM-2: adjust s/c efpa

% at this point, the main s/c/ is on a slower trajectory to the same EFPA
% with arrival time leadtime_tcm1 + leadtime_tcm2
%
% the main s/c gets data when the pathfinder (on original trajectory)
% arrives at EI, which is in leadtime_tcm1 days, and performs the maneuver

fp_time = leadtime_tcm1;

% forward propagate until the moment that pathfinder enters Neptune
[r2,v2] = TimeProp_V3(r1_tcm,v1_tcm,mu,fp_time);

% forward propagate until the moment that main s/c enters Neptune
[r_ei_2,v_ei_2] = TimeProp_V3(r1_tcm,v1_tcm,mu,fp_time + leadtime_tcm2);

% get current orbit parameters
[a2,e2,~,~,~,f2] = Get_Orb_Params(r2,v2,mu);

% get EI orbit parameters pre-TCM-2
[a_ei_2,e_ei_2,~,~,~,f_ei_2] = Get_Orb_Params(r_ei_2,v_ei_2,mu);

% optimize TCM-2 (x = [a,e,f])
ft =   @(x) 2*pi - acos( ( x(1)*(1-x(2)^2)-norm(r2) ) / x(2) / norm(r2) );
dfpa = @(x) abs( atan(   e2*sin( f2    ) / (1+  e2*cos( f2    )) ) - ...
                 atan( x(2)*sin( ft(x) ) / (1+x(2)*cos( ft(x) )) ) );
vt =   @(x) sqrt( mu * (2/norm(r2) - 1/x(1)) );
fun =  @(x) ( cos(dfpa(x)) * vt(x) - norm(v2) )^2 + ...
            ( sin(dfpa(x)) * vt(x) )^2;

x0 = [a_ei_2;norm(e_ei_2);f_ei_2];
lb = [-Inf,1,-2*asin(1/norm(e_ei_2))];
ub = [0,Inf,2*asin(1/norm(e_ei_2))];
options = optimoptions(@fmincon,'display','none');
[x,~,exitflag] = fmincon(fun,x0,[],[],[],[],lb,ub,...
                         @(x)nonlcon_tcm2(x,r_ei_2,r2,fpa_tgt),options);
a2_tcm = x(1);
e2_tcm = x(2);
f_ei_2_tcm = x(3);
r2_tcm = r2;

v2_tcm_norm = sqrt(mu*(2/norm(r2_tcm)-1/a2_tcm));

f2_tcm = acos((a2_tcm*(1-e2_tcm^2)/norm(r2_tcm)-1)/e2_tcm);
f2_tcm = 2*pi-sign(f2_tcm)*f2_tcm;

% find new velocity vector
fpa2 = atan(norm(e2)*sin(f2)/(1+norm(e2)*cos(f2)));
fpa2_tcm = atan(e2_tcm*sin(f2_tcm)/(1+e2_tcm*cos(f2_tcm)));

x_axis = r2_tcm / norm(r2_tcm);     % calculate the s/c coordinate frame
z_axis = [0;0;1];
y_axis = cross(z_axis,x_axis);
rotn = [x_axis, y_axis, z_axis];

v2_tcm = v2_tcm_norm * rotn * [sin(fpa2_tcm); cos(fpa2_tcm); 0];

dv2 = norm( v2 - v2_tcm );

disp(['TCM-2 Delta-v: ' num2str(dv2) ' m/s, Exit Flag: ' num2str(exitflag)])

if visualize

    figure(107)
    hold on
    
    disp_orbit(r_ei_0,v_ei_0,mu,50,0.05,0,'red')
    disp_orbit(r1_tcm,v1_tcm,mu,50,0.05,leadtime_tcm1+leadtime_tcm2,'magenta')
    disp_orbit(r2_tcm,v2_tcm,mu,50,0.05,leadtime_tcm2,'blue')

    [x,y,z] = sphere;
    x = x*neptune.r_m;
    y = y*neptune.r_m;
    z = z*neptune.r_m;
    nep_disp = surf(x,y,z);
    alpha(nep_disp,0.5)
    hold off

    view([1,1,1])
    
end

    
%% Nonlinear Constraint Functions

function [c,ceq] = nonlcon_tcm1(x,r_ei_0,r1,fpa_cur,dt,mu)

    a = x(1);
    e = x(2);
    f = x(3);
    cos_f1_tcm = (a*(1-e^2)-norm(r1))/norm(r1)/e;
    f1_tcm = 2*pi - acos(cos_f1_tcm);

    c(1) = a;  % SMA <= 0 (hyperbolic constraint)
    c(2) = -(e-1); % ECC >= 1 (hyperbolic constraint)
    c(3) = -(f-pi);  % f >= pi (negative fpa constraint)
    c(4) = abs(cos_f1_tcm)-1; % abs(cos(f1_tcm)) <= 1
    ceq(1) = norm(r_ei_0) - ( a * (1-e^2) ) / ( 1 + e*cos(f) );
    ceq(2) = tan(fpa_cur) - ( e*sin(f) / ( 1 + e*cos(f) ));
    
    E_tcm1 = 2 * atanh( sqrt((e-1)/(1+e)) * tan(f1_tcm/2) );
    E_ei   = 2 * atanh( sqrt((e-1)/(1+e)) * tan(     f/2) );
    
    M_tcm1 = e*sinh(E_tcm1) - E_tcm1; % main s/c mean anomaly at TCM-1
    M_ei   = e*sinh(E_ei  ) - E_ei;   % main s/c mean anomaly at EI
    
    ceq(3) = dt - (M_ei-M_tcm1)/sqrt(mu/(-a^3))/24/3600;

end

function [c,ceq] = nonlcon_tcm2(x,r_ei_2,r2,fpa_tgt)

    c(1) = x(1);  % SMA <= 0 (hyperbolic constraint)
    c(2) = -(x(2)-1); % ECC >= 1 (hyperbolic constraint)
    c(3) = x(3);  % f <= 0 (negative fpa constraint)
    c(4) = abs((x(1)*(1-x(2)^2)/norm(r2)-1)/x(2))-1; % abs(cos(f1_tcm)) <= 1
    ceq(1) = norm(r_ei_2) - ( x(1) * (1-x(2)^2) ) / ( 1 + x(2)*cos(x(3)) );
    ceq(2) = tan(fpa_tgt) - ( x(2)*sin(x(3)) / ( 1 + x(2)*cos(x(3)) ));
    ceq(3) = sign(fpa_tgt) - sign(x(3)); % sign(f) = sign(fpa)

end