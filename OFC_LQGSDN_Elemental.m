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
%           Remove passing of R,Q since recomputed anyway?
%           Absorb Elemental into Chunked code.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global tinit tmax Tgoal wgoal pgoal ngoal tsteps mdim xdim;
dimpg   = size(pgoal,2);
piE     = {};
KpiE    = {};
Chunks  = eye(ngoal,ngoal);

%% make copy of parameters
params = OFC_Parameters();                      % assumes that parameters have already been specified

%% solve elemental control problems
for g=1:ngoal
    tmax       = Tgoal(g);
    wgoal      = Chunks(g,:);                   % "switch on" single sub-goal only
    OFC_GlobalVars();                           % re-compute global variables
    [R,Q]      = OFC_LQG_costfunc();            % re-compute cost functions
    [pi,Kpi,V] = OFC_LQGSDN(x,A,B,C,H,O,R,Q);   % run optimization
    piE{g}     = pi;
    KpiE{g}    = Kpi;
    VE(g)      = V;
    x(1:dimpg) = squeeze(pgoal(g,:))';          % set xinit for next trial to current via-point target position
    tinit      = tmax;                          % set tinit for next trial to current via-point target time
end

%% restore parameters (tinit, tmax, wgoal)
OFC_Parameters('params',params);

%% stitch solutions together
pi      = nan(mdim,xdim,tsteps);
Kpi     = nan(xdim,xdim,tsteps);
TIinit  = 1;                                    % initial time index for each loop
for g=1:ngoal
    tstepsE     = size(KpiE{g},3);
    Ti          = TIinit:TIinit+tstepsE-1;      % very ugly, find a better way
    Kpi(:,:,Ti) = KpiE{g};
    pi(:,:,Ti)  = piE{g};
    TIinit      = TIinit + tstepsE;
end

%% compute cost-to-go
V = sum(VE);

end
