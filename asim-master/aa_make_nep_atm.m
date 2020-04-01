function nep_atm = aa_make_nep_atm
%AA_MAKE_NEP_ATM Creates a table of min, max, and nominal density values
%   These values can then be linearly interpolated using Fminmax = [-1,1]
%   to select an atmosphere profile.
%
% Author:   Tiger Hou
% Created:  02/07/2020
% Modified: 02/07/2020
%
% NOTES
%   - Must be placed in the directory asim-master
%
% SOURCES
%   - [1] Engineering-Level Model Atmospheres for Titan and Neptune
%   - [2] Atmospheric Models for Aerocapture Systems Studies

p = get_planetary_data(8,2);

nep_atm = p.atm.table;

% add Fminmax =  1 data to 3rd column
nep_atm(:,3) = nep_atm(:,2).*(10.^(nep_atm(:,1)/8e5));

% add Fminmax = -1 data to 4th column
nep_atm(:,4) = nep_atm(:,2)./(10.^(nep_atm(:,1)/4e5));

% add perturbation model to 5th column
% ratio provided in Fig 3(b) from [1]
LoH = 2;
% gravity of Neptune (at surface) divided by specifc heat constant
Cp = (p.atm.gamma * p.atm.R) / (p.atm.gamma - 1);
goCp = p.surface_g / Cp;
goR  = p.surface_g / p.atm.R;
% eqn.1 from [1]
% fudging the numbers a bit to match [1]
rho_ratio_pct = LoH/2/pi * sqrt(1+(LoH/2/pi)^2)...
                * (0.18*movmean(diff(nep_atm(:,4)),30) + goCp) / goR + 9;
nep_atm(2:end,5) = rho_ratio_pct / 100;

nep_atm = nep_atm(2:end,1:5);

end

