% lin_interp.m
%   Linear interpolation routine. Assumes domain is monotomically
%   increasing. Uses end points for xi outside the domain.
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
%   x - double(n,1), varies, domain for interpolation
%   y - double(n,m), varies, range for interpolation
%   xi - double(1), varies, point to perform interpolation at
%   
% Output:
%   yi - double(1,m), varies, interpolated values

function [yi] = lin_interp(x,y,xi)
%#codegen

n = length(x); % length of table

if xi <= x(1) % xi below domain

    % Use initial table value
    yi = y(1,:);

elseif xi >= x(n) % xi above domain

    % Use final table value
    yi = y(n,:);

else
    
    % Locate xi within x array (simple bisection method with index)
    jl = uint32(1);
    ju = uint32(n+1);
    jm = uint32(1);

    while ((ju-jl) > 1)
        jm = (ju+jl)/2;
        if ( xi >= x(jm) )
            jl = jm;
        else
            ju = jm;
        end
    end
    
    m = (y(ju,:) - y(jl,:)) / (x(ju)-x(jl));
    yi = m .* (xi-x(jl)) + y(jl,:);
    
end

end % lin_interp()