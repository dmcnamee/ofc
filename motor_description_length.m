function [D] = motor_description_length(G,varargin)
%% FUNCTION: Encoding complexity of motor plan based on description length.
% INPUTS:   G = optimal control gains
% OUTPUTS:  D = complexity measure
% NOTES:    Various options to be considered - continuous/discrete,
%           range normalization, weight by state distribution,
%           prior KL-DL, sequential KL-DL, adaptive compression...
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
%           Revised 19/04/2017: Hannah Sheahan, sheahan.hannah@gmail.com
%% variables
mdim    = size(G,1);            % dimensionality of plane of movement
xdim    = size(G,2);            % dimensionality of state variable X
tsteps  = size(G,3);            % number of time steps
nG      = mdim*xdim*tsteps;     % total number of gains
maxGain = max(G(:));
minGain = min(G(:));            % assume code normalizes within range
nq      = 100;                  % number of gain "quanta" in gain discretization
[pGain,GainEdges,Gidx] = histcounts(G(:),nq,'Normalization','probability');

%% settings
type = 'seqKLDL';

%% non-sequential DL
% sum gain description length over all plane x state x time
% gain distribution estimated from L
% assume uniform distribution over state-space (does not account for e.g. signal-dependent noise)

p = nan(nG,1); %p = nan(nG);  %**HRS(19/04/2017)
for i=1:nG
    p(i) = pGain(Gidx(i));
end

D = entropyPmtk(p); %D = entropy(p);  %**HRS(19/04/2017) The default matlab function takes RGB values and doesn't seem appropriate for this
figure();
histogram(G);

%% sequential KL-DL


end