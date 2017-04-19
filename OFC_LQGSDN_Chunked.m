function [pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q)
%% FUNCTION: OFC LQGSDN ViaPoint problem solved as sequence of chunked movements.
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
%           Remove passing of R,Q since recomputed anyway?
%           Enable 'Chunks' functionality for specifying only some of the between target
%           segments e.g. only the first movement
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global tinit tmax wgoal pgoal Tgoal mdim xdim tsteps;
dimpg       = size(pgoal,2);
nchunk      = size(Chunks,1);
piC         = {};
KpiC        = {};

%% make copy of parameters
params = OFC_Parameters();                      % assumes that parameters have already been specified

%% solve chunked control problems
for c=1:nchunk
    g           = find(Chunks(c,:),1,'last');
    tmax        = Tgoal(g);                         % set tmax to time of goal ending chunk
    wgoal       = Chunks(c,:);                      % "switch on" sub-goals in chunk only
    OFC_GlobalVars();                               % re-compute global variables
    [R,Q]       = OFC_LQG_costfunc();               % re-compute cost functions
    [pi,Kpi,V]  = OFC_LQGSDN(x,A,B,C,H,O,R,Q);      % run optimization
    piC{c}      = pi;
    KpiC{c}     = Kpi;
    VC(c)       = V;
%     x(1:dimpg)  = squeeze(pgoal(g,:))';             % set xinit for next trial to current via-point target position
    [TX,~]      = OFC_RollOut(x,pi,Kpi,A,B,H,R,Q);  % roll out control policy
    x(1:3*mdim) = TX(1:3*mdim,end);                 % set init for next element to final state of current
    tinit       = tmax;                             % set tinit for next trial to current via-point target time
end

%% restore parameters (tinit, tmax, wgoal)
OFC_Parameters('params',params);

%% stitch solutions together
pi      = nan(mdim,xdim,tsteps);
Kpi     = nan(xdim,xdim,tsteps);
TIinit  = 1; % initial time index for each loop
for c=1:nchunk
    tstepsC     = size(KpiC{c},3);
    Ti          = TIinit:TIinit+tstepsC-1;      % very ugly, find a better way
    Kpi(:,:,Ti) = KpiC{c};
    pi(:,:,Ti)  = piC{c};
    TIinit      = TIinit + tstepsC;
end

%% compute cost-to-go
V = sum(VC);

end
