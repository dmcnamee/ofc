function vars = OFC_GlobalVars()
%% FUNCTION: Globalizes and returns DEPENDENT global variables for OFC.
% INPUTS:   varargin    = {pass parameters explicitly}
% OUTPUTS:  T           = discretized time-space
%           tsteps      = time resolution
%           psteps      = position resolution
%           vsteps      = velocity resolution
%           usteps      = control signal resolution
%           ngoal       = number of goal-targets
%           xdim        = dimensionality of state variable x (incl. sensorimotor delay)
%           smdsteps    = number of discrete time steps in sensorimotor delay
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
global smdelay smdsteps xinit;

%% dependent variables
T           = tinit:tres:tmax;
tsteps      = numel(T);
psteps      = (plim(:,2)-plim(:,1))/pres + 1;
vsteps      = (vlim(:,2)-vlim(:,1))/vres + 1;
usteps      = (ulim(:,2)-ulim(:,1))/ures + 1;
ngoal       = size(pgoal,1);
smdsteps    = ceil(smdelay/tres);  
xdim        = 3*mdim +smdsteps*3*mdim + ngoal*mdim;                             % current state, delayed feedback states, task goals
if numel(xinit) ~= xdim
    xinit       = [repmat(xinit(1:3*mdim),smdsteps+1,1); xinit(3*mdim+1:end)];  % adapt xinit to incorporate smdelay
end

end