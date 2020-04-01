% guidance_init.m 
%   Initializes guidance data structures
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
%   
% Output:
%   guid - data structure, misc, guidance data structure with parameter
%       values set

function [guid] = guidance_init( in )
%#codegen

%% Initialize data structure
guid = define_guid;

%% Populate algorithm-specific parameter data structures
guid.msl.p = in.v.gnc.g.p_msl;
guid.sej_a.p = in.v.gnc.g.p_sej_a;
guid.dma_cv.p = in.v.gnc.g.p_dma_cv;
guid.sej_e.p = in.v.gnc.g.p_sej_e;
guid.gt.p = in.v.gnc.g.p_gt;


%% Guidance parameters
guid.p = in.v.gnc.g.p;

%% Guidance states
guid.s.init_flag = logical(true); % set initialization flag to true for first call to guidance
guid.s.sg_ratio = round(in.s.traj.rate/guid.p.rate); % sim/control rate ratio, nd

%% Intialize command structure
guid.cmd.bank = guid.p.const_bank;
guid.cmd.bank_rate = guid.p.const_bank_rate;
guid.cmd.aoa = guid.p.const_aoa;

end % guidance_init
