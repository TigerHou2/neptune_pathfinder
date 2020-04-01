% leading_zeros.m 
%   Utility function to add leading zeros to integers for file naming
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
%   num - int32(1), nd, number to add leading zeros to
%   len - int32(1), nd, desired string length
%   
% Output:
%   str - string(len), nd, string containing input num with leading zeros

function str = leading_zeros( num, len )
    
    % Initialize string
    str = '';
   
    % check length of num relative to desired length
    if num < (10^(len-1))
        % Add a zero and call again
        str = ['0' leading_zeros( num, len-1)];
    else
        % no zero required--return str
        str = int2str(num);
    end
    
end % leading_zeros