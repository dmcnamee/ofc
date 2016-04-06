function vars = OFC_GlobalVars()
%% FUNCTION: Globalizes and returns DEPENDENT global variables for OFC.
% INPUTS:   varargin  = {pass parameters explicitly}
% OUTPUTS:  T       = discretized time-space
%           tsteps  = time resolution
%           psteps  = position resolution
%           vsteps  = velocity resolution
%           usteps  = control signal resolution
%           ngoal   = number of goal-targets
%           xdim    = dimensionality of state variable x
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002 / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% set parameters/variables as global
% clearvars -global;
global tinit mdim;
global plim vlim ulim alim;
global tres pres vres ures;
global pgoal ngoal;
global tsteps psteps vsteps usteps;
global tmax;
global xdim;
global T;

%% dependent variables
T       = tinit:tres:tmax;
tsteps  = numel(T);
psteps  = (plim(:,2)-plim(:,1))/pres + 1;
vsteps  = (vlim(:,2)-vlim(:,1))/vres + 1;
usteps  = (ulim(:,2)-ulim(:,1))/ures + 1;
ngoal   = size(pgoal,1);
xdim    = mdim*(3+ngoal);
    
end