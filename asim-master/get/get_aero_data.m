% get_aero_data.m 
%   Returns aerodynamics data based on vehicle state
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
%   mach - double(1), nd, current Mach number
%   veh - struct(1), multi, vehicle data structure
%       s.aoa - double(1), rad, current angle-of-attack
%       s.ssa - double(1), rad, sideslip angle
%       s.compute_trim - uint8(1), nd, trim angle-of-attack calculation flag
%   alt - double(1), m, current altitude
%   v_wind_mag - double(1), m, current altitude
%   dynp - double(1), Pa, current dynamic pressure
%   cg - double(3), m, current c.g. position
%   in - struct(1), md, input data structure
%   
% Output:
%   cl - double(1), nd, vehicle lift coefficient at angle of attack
%   cd - double(1), nd, vehicle drag coefficient at angle of attack
%   aoa - double(1), rad, current angle-of-attack

function [cl, cd, veh] = get_aero_data(mach, veh, alt, ...
    v_wind_mag, dynp, cg, in)
%#codegen

%% Initialize outputs
cl = 0;
cd = 0;


%% Aerodynamics modes
switch in.v.aero.mode
    
    case 1 
    %% Constant aerodynamic coefficients
        
        cl = in.v.aero.cl;
        cd = in.v.aero.cd;
        
        
    case 2
    %% Mach-dependent table look-up aerodynamic coefficients

        clcd = lin_interp(in.v.aero.table(:,1), in.v.aero.table(:,2:3), mach);
        cl = clcd(1);
        cd = clcd(2);        
        if isnan(cl)
            cl = 0;
            cd = 0;
        end
        
        
    otherwise
    %% Use constant coefficients
    
        cl = in.v.aero.cl;
        cd = in.v.aero.cd;

        
end

end % get_aero_data()