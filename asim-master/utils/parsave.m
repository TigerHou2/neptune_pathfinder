% parsave.m
%   Wrapper for "save" function; required for parallel computing
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
%   fname - string, nd, file name (including relative path) for save
%   dname - string, nd, desired variable name for save
%   data - multi, md, data to save
%   
% Output:
%   none.

function parsave( fname, dname, data )

    eval([dname '= data;']);
    
    save( fname, dname );

end