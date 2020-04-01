% get_atm_data.m 
%   Returns atmospheric data based on current vehicle state.
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
%   in - data structure, misc, input data structure
%   alt - double(1), m, altitude above the reference ellipsoid
%   
% Output:
%   dens - double(1), kg/m^3, density at altitude
%   pres - double(1), Pa, pressure at altitude
%   temp - double(1), K, temperature at altitude
%   wind - double(3), m/s, E,N,vert winds

function [dens, pres, temp, wind] = get_atm_data(in, alt)
%#codegen

switch in.p.atm.mode
    
    case 1 % Exponential model
        pres = in.p.atm.pres_ref*exp(-alt/in.p.atm.scale_height);
        dens = in.p.atm.dens_ref*exp(-alt/in.p.atm.scale_height);
        temp = in.p.atm.temp_ref;
        wind = [0 0 0];  % east; north; vertical

    case 2 % Table look-up model
               
        atm = lin_interp(in.p.atm.table(:,1), in.p.atm.table(:,2:4), alt);
        dens = atm(1);
        pres = atm(2);
        temp = atm(3);
        wind = [0 0 0];  % east; north; vertical
        if isnan(dens)
            dens = 0;
            pres = 0;
            temp = 0;
        end
        
    case 3 % Table look-up model with winds
        
        atm = lin_interp(in.p.atm.table(:,1), in.p.atm.table(:,2:7), alt);
        dens = atm(1);
        pres = atm(2);
        temp = atm(3);
        wind = [atm(4) atm(5) atm(6)];  % east; north; vertical
        if isnan(dens)
            dens = 0;
            pres = 0;
            temp = 0;
        end
    
    otherwise
    
        dens = 0;
        pres = 0;
        temp = 0;
        wind = [0 0 0];  % east; north; vertical
end

end % get_atm_data()