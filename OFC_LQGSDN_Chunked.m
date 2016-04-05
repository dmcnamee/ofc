function [pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q)
%% FUNCTION: OFC LQGSDN ViaPoint problem solved as sequence of elemental movements.
% INPUTS:   Chunks = indicator matrix describing chunks/elementals (#chunks x #goals)
%           x = initial state of system
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
%           Update parameter management after outputing parameters as
%           struct - clearing globalvar making things awkward higher up.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global pgoal ngoal Tgoal;
dimpg       = size(pgoal,2);
nchunk      = size(Chunks,1);
piC         = {};
KpiC        = {};

%% make copies of variables describing all task constraints
pgoalA = pgoal;
ngoalA = ngoal;
TgoalA = Tgoal;

%% solve chunked control problems
tinit = 0;
for c=1:nchunk
    g          = find(Chunks(c,:),1,'last');
    tmaxC      = TgoalA(g);
    wgoalC     = Chunks(c,:);  % "switch on" sub-goals in chunk
    OFC_Parameters('tinit',tinit,'tmax',tmaxC,'wgoal',wgoalC,'pgoal',pgoalA,'Tgoal',TgoalA);
    [R,Q]      = OFC_LQG_costfunc();
    [pi,Kpi,V] = OFC_LQGSDN(x,A,B,C,H,O,R,Q);
    piC{c}     = pi;
    KpiC{c}    = Kpi;
    VC(c)      = V;
    x(1:dimpg) = squeeze(pgoalA(g,:));    % set xinit for next trial to current via-point target position
    tinit      = tmaxC;                   % set tinit for next trial to current via-point target time
end

%% stitch solutions together
OFC_Parameters('pgoal',pgoalA,'Tgoal',TgoalA);
global tsteps mdim xdim;
pi      = nan(tsteps,mdim,xdim);
Kpi     = nan(tsteps,xdim,xdim);
TIinit  = 1; % initial time index for each loop
for c=1:nchunk
    tstepsC     = size(KpiC{c},1);
    Ti          = TIinit:TIinit+tstepsC-1;  % very ugly, find a better way
    Kpi(Ti,:,:) = KpiC{c};
    pi(Ti,:,:)  = piC{c};
    TIinit      = TIinit + tstepsC;
end

%% compute cost-to-go
V = sum(VC);

end
