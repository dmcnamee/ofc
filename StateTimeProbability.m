function [P,edges] = StateTimeProbability(xinit,L,K,A,B,C,H,O,sigma,varargin)
%% FUNCTION: Monte-Carlo estimate of state-time probability.
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
% OUTPUTS:  PX          = probability x state matrix
% NOTES:    N/A
% ISSUES:   Large array error.
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
% generates large array error (understandably...)
Tflat = reshape(T,mdim*3*tsteps,[]);
[N, edges, ~, ~] = histcn(Tflat');
% N = reshape...

%% probability dist
P  = (N+eps)./max(N+eps);

end