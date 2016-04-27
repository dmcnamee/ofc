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
while ~isempty(varargin)
    switch varargin{1}
        case 'Perturbations'
            Pert = varargin{2};
        case 'Plot'
            plot = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
      varargin(1:2) = [];
end
if ~exist('plot','var')
    plot = false;
end
global tsteps T mdim xdim;
TX = zeros(xdim,tsteps);    % discretized time/state-space
if numel(x)==mdim           % extend to include target positions
    global xinit;
    x = [x xinit(3*mdim+1:xdim)]';
end

%% init time/state-space
TX(:,1)        = x;
if exist('Pert','var')
    [TX(:,1),Pert] = OFC_ApplyPerturbation(1,TX(:,1),Pert);         % apply perturbation
end

for ti=2:tsteps
    Lt          = squeeze(L(:,:,ti));                               % corresponding feedback control gain
    x           = TX(:,ti-1);                                       % set current state
    % optimal estimation using Kalman filter
    u           = -Lt*x;                                            % derive control input from control law
    TX(:,ti)    = A*x + B*u;                                        % update next state, no sampled noise
    if exist('Pert','var')
        [TX(:,ti),Pert] = OFC_ApplyPerturbation(ti,TX(:,ti),Pert);  % apply tstep-state perturbation
    end
    
    if any(isnan(TX(:)))
        error('OFC_RollOut: nan returned.');
    end
end

%% compute cost trajectory
QX = zeros(1,tsteps);
for ti=1:tsteps
    Qt     = OFC_TaskConstraintCostMatrix(T(ti),Q);
    QX(ti) = OFC_ComputeCost(TX(:,ti),R,Qt);
end

%% output diagnostics
fprintf('OFC_RollOut: cumulative cost = %.3f.\n',sum(QX));

%% plot
if plot
    OFC_Plot(TX,varargin{:});
end
