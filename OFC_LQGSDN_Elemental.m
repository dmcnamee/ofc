function [pi,Kpi,V] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q)
%% FUNCTION: OFC LQGSDN ViaPoint problem solved as sequence of elemental movements.
% INPUTS:   x = initial state of system
%           A = state-evolution matrix
%           B = control-input matrix
%           C = signal-dependent noise matrix
%           H = sensory-indicator matrix
%           O = sensory-noise-covariance matrix
%           R = energy-cost matrix
%           Q = goal-targets-cost matrix
% OUTPUTS:  pi  = optimal control law   (LQR)
%           Kpi = optimal Kalman filter (LQE)
%           V   = expected cost-to-go
% NOTES:    N/A
% ISSUES:   Need to re-compute velocity/position based on stitched elemental LQRs.
%           Update parameter management after outputing parameters as struct.
%           Admit non-trivial chunking structure.
%           Admit perturbations (zero Wstop?).
%           Use wgoal instead of setting pgoal explicitly.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global pgoal ngoal Tgoal;
dimpg   = size(pgoal,2);
piE     = {};
KpiE    = {};

%% make copies of variables describing all task constraints
pgoalA = pgoal;
ngoalA = ngoal;
TgoalA = Tgoal;

%% solve elemental control problems
tinit = 0;
for g=1:ngoal
    tmaxE    = TgoalA(g);
    pgoalE   = squeeze(pgoalA(g,:));
    wgoalE   = zeros(1,ngoalA); wgoalE(g) = 1; % switch sub-goal on
    OFC_Parameters('tinit',tinit,'tmax',tmaxE,'wgoal',wgoalE,'pgoal',pgoalA,'Tgoal',TgoalA);
    [R,Q]    = OFC_LQG_costfunc();
    [pi,Kpi,V] = OFC_LQGSDN(x,A,B,C,H,O,R,Q);
    piE{g}     = pi;
    KpiE{g}    = Kpi;
    VE(g)      = V;
    x(1:dimpg) = pgoalE;       % set xinit for next trial to current via-point target position
    tinit      = tmaxE;        % set tinit for next trial to current via-point target time
end

%% stitch solutions together
OFC_Parameters('pgoal',pgoalA);
global ngoal tsteps mdim xdim;
pi      = nan(tsteps,mdim,xdim);
Kpi     = nan(tsteps,xdim,xdim);
TIinit  = 1; % initial time index for each loop
for g=1:ngoal
    tstepsE     = size(KpiE{g},1);
    Ti          = TIinit:TIinit+tstepsE-1;  % very ugly, find a better way
    Kpi(Ti,:,:) = KpiE{g};
    pi(Ti,:,:)  = piE{g};
    TIinit      = TIinit + tstepsE;
end

%% compute cost-to-go
V = sum(VE);

end
