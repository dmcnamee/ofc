function [TX,QX] = OFC_RollOut(x,L,K,A,B,H,R,Q,varargin)
%% FUNCTION: OFC LQG control law rollout.
% INPUTS:   x           = initial state
%           L           = control law
%           K           = Kalman filter
%           A           = state-evolution matrix
%           B           = control-input matrix
%           H           = sensory-indicator matrix
%           R           = energy cost matrix
%           Q           = goal-targets-costs matrix
%           Pert.TX     = explicit tstep-state specification of perturbations (tsteps x 3*mdim)
%           Pert.Th.T   = threshold for T constraints
%           Pert.Th.X   = threshold for X constraints
%           Pert.C.T    = time constraint (e.g. time-t'<Th.T)
%           Pert.C.X    = state constraint (e.g. state-x'<Th.X)
%           Pert.P.T    = perturbation corresponding to time constraint
%           Pert.P.X    = perturbation corresponding to state constraint
% OUTPUTS:  TX          = time x state matrix
%           QX          = time x cost matrix
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global tsteps T;

%% roll out trajectory
TX = TrajectoryRollOut(x,L,K,A,B,H,varargin{:});

%% compute cost trajectory
QX = zeros(1,tsteps);
for ti=1:tsteps
    Qt     = OFC_TaskConstraintCostMatrix(T(ti),Q);
    QX(ti) = OFC_ComputeCost(TX(:,ti),R,Qt);
end

%% output diagnostics
fprintf('OFC_RollOut: cumulative cost = %.3f.\n',sum(QX));

end