function [P,edges,midpoints] = StatePDF(xinit,L,K,A,B,C,H,O,sigma,varargin)
%% FUNCTION: Sampling estimate of state probability density.
% INPUTS:   x           = initial state
%           L           = control law
%           K           = Kalman filter
%           A           = state-evolution matrix
%           B           = control-input matrix
%           C           = signal-dependent noise
%           H           = sensory-indicator matrix
%           O           = sensory noise covariance matrix ("Omega")
%           sigma       = variance of Wiener process
%           Pert.TX     = explicit tstep-state specification of perturbations (tsteps x 3*mdim)
%           Pert.Th.T   = threshold for T constraints
%           Pert.Th.X   = threshold for X constraints
%           Pert.C.T    = time constraint (e.g. time-t'<Th.T)
%           Pert.C.X    = state constraint (e.g. state-x'<Th.X)
%           Pert.P.T    = perturbation corresponding to time constraint
%           Pert.P.X    = perturbation corresponding to state constraint
% OUTPUTS:  P           = probability x state matrix
%           edges       = grid edges used for binning statespace
%           midpoints   = midpoints of statespace grid
% NOTES:    This is really dumb - better way? Directly approximate entropy?
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% settings
nsamp = 1000;

%% variables
global mdim xdim tsteps;
T = nan(mdim*3,tsteps,nsamp);

%% roll out expected trajectory
for s=1:nsamp
    TX = TrajectorySample(xinit,L,K,A,B,C,H,O,sigma,varargin{:});
    T(1:6,:,s) = TX(1:6,:);
end

%% compute probability
Tflat = reshape(T,mdim*3,[]);
[N, edges, midpoints, ~] = histcn(Tflat');

%% probability dist
P  = (N+eps)./max(N+eps);

end