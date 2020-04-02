%% aa_bank_mod_v3_deltav
%
% Author:   Tiger Hou
% Created:  02/27/2020
% Modified: 02/28/2020
%
% DESCRIPTION
%   - This script simulates aerocapture at Neptune using bank angle control
%       Variables include L/D, V0, EFPA, and Ap Tolerance
%   - For each configuration, two trajectories are flown (fully lift-down
%       and fully lift-up) to find the maximum boundaries.
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
in = aa_bank_mod_v3_deltav_in();    % load the input file


disp('Initialization complete!')
disp('')


%% Data Saving Options

% get filename for data file
opts.Interpreter = 'tex';
fName = inputdlg( '\fontsize{12} Enter data file name', ...
                  'User Input', [1,50], ...
                 {'data_file'}, opts);
if isempty(fName)
    warning('No file name provided. Using default name "data_file.mat",.');
    fName = 'data_file';
else
    fName = fName{1};
end

%% Run Simulation

disp('Starting sim...')

% orbiting body gravitational parameter
mu = in.p.mu;

% simulation ranges
ld_range   = [0.15,0.2,0.25,0.3,0.4];
v_init     = (26:0.25:35)*1e3;
gammas     = -12:0.05:-5.0;

% get size of each dimension
ld_len = length(ld_range);
vi_len = length(v_init);
ga_len = length(gammas);
datsize = ld_len*vi_len*ga_len;

disp(['Data size: ' num2str(datsize) ...
      ' | Estimated time: ' num2str(datsize/5) ' seconds'])
disp('')

in_cell = cell(ld_len,vi_len,ga_len);

for i = 1:ld_len
    for j = 1:vi_len
        for k = 1:ga_len
            in_cell{i,j,k} = [ld_range(i),v_init(j),gammas(k)];
        end
    end
end

result = NaN(datsize,7);

parfor i = 1:datsize
    % start simulation
    disp(['Progress: ' num2str(i)])
    temp = in_cell{i};
    LoD  = temp(1);
    v0   = temp(2);
    fpa0 = temp(3);
    
    this_in = in;
    
    this_in.v.aero.cd = in.v.aero.cl / LoD;
    this_in.s.traj.v_pci_ini = v0 .* [sind(fpa0); cosd(fpa0); 0];
    this_in.v.gnc.g.p_sej_a.tgt_ap = 430000e3;
    this_in.v.gnc.g.p.const_bank = pi;
    
    % ======================  lift-up trajectory =========================
    this_in.v.gnc.g.p.const_bank = 0; % rad, bank angle
    dat_temp = aa_main_mex(this_in);
    
    % calculate semi-major axis of orbit
    idx = sum(~isnan(dat_temp.traj.time)); % termination index
    r = dat_temp.traj.pos_ii(idx,:)';      % termination position
    v = dat_temp.traj.vel_ii(idx,:)';      % termination velocity
    [a_up,e_up] = Get_Orb_Params(r,v,mu);
    
    % ======================  lift-down trajectory =======================
    this_in.v.gnc.g.p.const_bank = pi; % rad, bank angle
    dat_temp = aa_main_mex(this_in);
    
    % calculate semi-major axis of orbit
    idx = sum(~isnan(dat_temp.traj.time)); % termination index
    r = dat_temp.traj.pos_ii(idx,:)';      % termination position
    v = dat_temp.traj.vel_ii(idx,:)';      % termination velocity
    [a_down,e_down] = Get_Orb_Params(r,v,mu);
    
    % save to results
    result(i,:) = [LoD, v0, fpa0, a_up, norm(e_up), a_down, norm(e_down)];
end

save(['aa-contours/' fName '.mat'], ...
    'result', 'mu', ...
    'ld_range', 'v_init', 'gammas', ...
    'ld_len', 'vi_len', 'ga_len');

disp('Simulation complete!')
disp('')

%% Plot Acceptable Configurations by Delta-V Constraint

target = [430000e3, 3986e3 + 24764e3]; % target orbit Ap and Pe, [3] p.7

dv_lim = 450; % m/s

% transfer results to temporary array
out = result;

% remove configurations where full lift-down generates a hyperbolic orbit
out(out(:,6)<0,:) = [];

% filter orbits by type:
% ** we are assuming the Pe correction burn dv is negligible compared to Ap
%   1 - orbits whose Ap bounds contain the target Ap
%   2 - orbits whose max Ap bound is lower than the target Ap
%   3 - orbits whose min Ap bound is higher than the target Ap

% get Ap and Pe of post-aerocapture orbit
init_orbit_up = [out(:,4).*(1+out(:,5)), out(:,4).*(1-out(:,5))]; % m, [Ap,Pe]
init_orbit_dn = [out(:,6).*(1+out(:,7)), out(:,6).*(1-out(:,7))]; % m, [Ap,Pe]

init_orbit = [init_orbit_up, init_orbit_dn];
out = [out, init_orbit];

out_1 = out;
out_2 = out;
out_3 = out;

% case 1
idx1 = 0<out_1(:,8);
idx2 = out_1(:,8)<target(1);
idx = logical(idx1 .* idx2);
out_1(idx,:) = []; % remove 0 < max(Ap) < tgt(Ap) (keeps hyperbolic cases)
out_1(out_1(:,10)>target(1),:) = []; % remove min(Ap) > tgt(Ap)
% case 2
out_2(out_2(:,8)>=target(1),:) = []; % remove max(Ap) >= tgt(Ap)
out_2(out_2(:,8)<=0,:) = []; % remove max(Ap) = hyperbolic cases
% case 3
out_3(out_3(:,10)<=target(1),:) = []; % remove min(Ap) <= tgt(Ap)

% CASE 1: 
%   the proper apoapsis can be achieved without a correction
%   only need to calculate minimum Pe correction burn
%   uses choice of Pe from bounds that minimizes difference from tgt Pe
%       (because maximizing Pe difference will result in Pe inside planet)
if isempty(out_1)
    out_1 = NaN([0,size(out,2)]);
end
v_ap_tgt = sqrt( mu * ( 2/target(1) - 2/(target(1)+target(2)) ) );
% maneuver at Ap to target Pe, lift-up case
v_ap_1_1 = sqrt( mu .* ( 2./target(1) - 2./(target(1)+out_1(:,9)) ) );
dv_1_1 = abs(v_ap_tgt-v_ap_1_1);
% maneuver at Ap to target Pe, lift-down case
v_ap_1_2 = sqrt( mu .* ( 2./target(1) - 2./(target(1)+out_1(:,11)) ) );
dv_1_2 = abs(v_ap_tgt-v_ap_1_2);
% compare cases and find minimum
dv_1 = min([dv_1_1,dv_1_2],[],2);
out_1 = [out_1,dv_1];

% CASE 2:
%   calculate delta-v from max Ap case
if isempty(out_2)
    out_2 = NaN([0,size(out,2)]);
end
%   maneuver at Ap to target Pe
v_ap_2   = sqrt( mu .* ( 2./out_2(:,8) - 2./(out_2(:,8)+out_2(:,9)) ) );
v_ap_tgt = sqrt( mu .* ( 2./out_2(:,8) - 2./(out_2(:,8)+target(2))  ) );
%   maneuver at (adujsted) Pe to target Ap
v_pe_2   = sqrt( mu .* ( 2./target(2)  - 2./(out_2(:,8)+target(2)) ) );
v_pe_tgt = sqrt( mu .* ( 2./target(2)  - 2./(target(1) +target(2)) ) );
%   sum delta-v costs of both maneuvers
dv_2 = abs(v_ap_tgt-v_ap_2) + abs(v_pe_tgt-v_pe_2);
out_2 = [out_2,dv_2];

% CASE 3:
%   calculate delta-v from min Ap case
if isempty(out_3)
    out_3 = NaN([0,size(out,2)]);
end
%   maneuver at Ap to target Pe
v_ap_3   = sqrt( mu .* ( 2./out_3(:,10) - 2./(out_3(:,10)+out_3(:,11)) ) );
v_ap_tgt = sqrt( mu .* ( 2./out_3(:,8)  - 2./(out_3(:,8) +target(2))   ) );
%   maneuver at (adujsted) Pe to target Ap
v_pe_3   = sqrt( mu .* ( 2./target(2)  - 2./(out_3(:,10)+target(2)) ) );
v_pe_tgt = sqrt( mu .* ( 2./target(2)  - 2./(target(1)  +target(2)) ) );
%   sum delta-v costs of both maneuvers
dv_3 = abs(v_ap_tgt-v_ap_3) + abs(v_pe_tgt-v_pe_3);
out_3 = [out_3,dv_3];

% recombine all cases
out = [out_1;out_2;out_3];
% out = out_1;

% remove all results that exceed the delta-v limit
out(min(out(:,end),[],2)>dv_lim,:) = [];

% create temporary cell to hold results (currently in 'out') sorted by L/D
temp = cell(ld_len,1);

% sort results by L/D
for i = 1:ld_len
    tmp = out;
    tmp(tmp(:,1)~=ld_range(i),:) = [];
    temp{i} = tmp;
end

figure
hold on
lgd = {};
for i = 1:ld_len
    tmp = temp{i};
    scatter3(tmp(:,2)/1e3,tmp(:,3),tmp(:,end))
    lgd{end+1} = strcat('L/D = ',num2str(ld_range(i)));
end
hold off

grid on
title('\textbf{Entry Theoretical Corridor Width}',...
        'interpreter','latex','fontweight','bold','fontsize',18)
xlabel('\textbf{Initial Velocity, km/s}',...
        'interpreter','latex','fontweight','bold','fontsize',18)
ylabel('\textbf{Flight Path Angle, deg}',...
        'interpreter','latex','fontweight','bold','fontsize',18)
zlabel('\textbf{Orbit Correction Delta-V Cost, m/s}',...
        'interpreter','latex','fontweight','bold','fontsize',18)
h = legend(lgd);
set(h,'location','best',...
        'interpreter','latex','fontweight','bold','fontsize',18)
set(gca,'TickLabelInterpreter','latex','fontweight','bold','fontsize',18)
set(gcf,'Color',[1 1 1])
set(gca,'Color',[1 1 1])
plotbrowser('on')


%% Single Test Run

close all hidden
clear;clc
setup
in = aa_bank_mod_v2_in();    % load the input file

disp('Initialization complete!')
disp('')

% test = aa_main_mex(in);
test = aa_main(in);

idx = sum(~isnan(test.traj.time)); % find index of termination point

r = test.traj.pos_ii(idx,:)'; % position vector [m]
v = test.traj.vel_ii(idx,:)'; % velocity vector [m/s]

res = 100;  % calculate 100 orbit points for hyperbolic orbits [nd]
start_day = 0.0001; % set the middle point of the orbit to be periapsis [days]
mu = in.p.mu; % get gravitational parameter of orbiting body [m^3/s^2]

ah = norm(r)/(2 - norm(r)*dot(v,v)/mu); % calculate semimajor axis length [m]
disp(ah)


%% Plot Orbit for Test Run

if ah >= 0
    dur = 2*pi*sqrt(ah^3/mu)/3600/24;  % orbit propagation duration [days]
else
    dur = 0.1;  % hyperbolic. plot 0.1 days centered around periapsis
%     ap = in.v.gnc.g.p_sej_a.tgt_ap;
%     dur = 2*pi*sqrt(ap^3/mu)/3600/24;
end

pos = Get_Orb_Points(r,v,mu,res,dur,start_day);
e_vec = ((dot(v,v)-mu/norm(r))*r - dot(r,v)*v)/mu;

hold on

[x,y,z] = sphere;
x = x*in.p.r_m;
y = y*in.p.r_m;
z = z*in.p.r_m;
surf(x,y,z)
plot3(pos(:,1), pos(:,2), pos(:,3),'LineWidth',2.5,'Color','Red')

% view(cross(r,v))
view([1,1,1])
pbaspect([1 1 1])
axis equal
grid on
hold off