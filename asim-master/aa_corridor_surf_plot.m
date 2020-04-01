function aa_corridor_surf_plot(varargin)
%AA_CORRIDOR_SURF_PLOT Plots entry corridor as a fcn of B2/B1 or L/D
%   
% Author:   Tiger Hou
% Created:  02/07/2020
% Modified: 02/09/2020
%

% this function can optionally take in the filename as argument
p = inputParser;
addParameter(p,'filename','');
parse(p,varargin{:});

% if filename is provided, proceed. Otherwise, prompt user selection
if ~isempty(p.Results.filename)
    filename = [p.Results.filename '.mat'];
    filepath = '.\aa-contours\';
else
    [filename,filepath] = uigetfile('.\aa-contours\*.mat');
end
if not(filepath)
    disp('No file selected.')
    return
end
dat = load(fullfile(filepath,filename));

% transfer data into local variables
in = dat.in;
apoapsis_result = dat.apoapsis_result;
beta_ratio = dat.beta_ratio;
v_init = dat.v_init;
gammas = dat.gammas;
br_len = dat.br_len;
vi_len = dat.vi_len;
ga_len = dat.ga_len;

% reshape the map into 3D, with each dimention representing
% beta ratio, initial velocity, and flight path angle
apoapsis_map = reshape(apoapsis_result,[br_len,vi_len,ga_len]);
% find all apoapsis results within tolerances
apoapsis_map(apoapsis_map > (in.v.gnc.g.p_sej_a.tgt_ap + ...
                             in.v.gnc.g.p_sej_a.tol_ap)) = 0;
apoapsis_map(apoapsis_map < (in.v.gnc.g.p_sej_a.tgt_ap - ...
                             in.v.gnc.g.p_sej_a.tol_ap)) = 0;
apoapsis_map(apoapsis_map > 0) = 1;

% find the x,y,z indices of viable initial states
[brInd,viInd,gaInd]=ind2sub(size(apoapsis_map),find(apoapsis_map));
% convert initial state indices to values
brVals = zeros(size(brInd));
for i = 1:length(brInd)
    brVals(i) = beta_ratio(brInd(i));
end
viVals = zeros(size(viInd));
for i = 1:length(viInd)
    viVals(i) = v_init(viInd(i));
end
gaVals = zeros(size(gaInd));
for i = 1:length(gaInd)
    gaVals(i) = gammas(gaInd(i));
end
% plot accepted orbits as 3D points
figure(1)
scatter3(brVals, viVals/1e3, gaVals);
grid(gca,'minor')
grid on
pbaspect([1 1 1])
xlabel('Beta Ratio')
ylabel('Initial Velocity (km/s)')
zlabel('Flight Path Angle (deg)')

% plot above results as 3D surface
figure(2)
tri = delaunay(brVals,viVals/1e3);
trisurf(tri,brVals,viVals/1e3,gaVals);
grid on
pbaspect([1 1 1])
lighting phong
% shading interp
xlabel('Beta Ratio')
ylabel('Initial Velocity (km/s)')
zlabel('Flight Path Angle (deg)')

% plot corridor width on z-axis
figure(3)
corr_width_map = zeros(br_len,vi_len);
for i = 1:br_len
    for j = 1:vi_len
        % find all valid FPA indices
        %   for each combination of B2/B1 (or L/D) and v_init
        validPos = find(squeeze(apoapsis_map(i,j,:)));
        if length(validPos)>1
            gaMin = gammas(min(validPos));
            gaMax = gammas(max(validPos));
            corr_width_map(i,j) = gaMax - gaMin;
        else
            corr_width_map(i,j) = 0;
        end
    end
end

[viGrid,brGrid] = meshgrid(v_init,beta_ratio);
scatter3(viGrid(:)/1e3,brGrid(:),corr_width_map(:));
grid(gca,'minor')
grid on
pbaspect([1 1 1])
xlabel('Initial Velocity (km/s)')
ylabel('Beta Ratio')
zlabel('Corridor Width (deg)')

end

