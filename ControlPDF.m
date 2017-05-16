function [pU,edges,midpoints] = ControlPDF(xinit,L,K,A,B,C,H,O,sigma,varargin)
%% FUNCTION: Monte-Carlo estimate of control output variable probability density.
% INPUTS:   x           = initial state
%           L           = control law
%           K           = Kalman filter
%           A           = state-evolution matrix
%           B           = control-input matrix
%           C           = signal-dependent noise
%           H           = sensory-indicator matrix
%           O           = sensory noise covariance matrix ("Omega")
%           sigma       = variance of Wiener process
%           nsamp       = number of samples to run
%           Pert.TX     = explicit tstep-state specification of perturbations (tsteps x 3*mdim)
%           Pert.Th.T   = threshold for T constraints
%           Pert.Th.X   = threshold for X constraints
%           Pert.C.T    = time constraint (e.g. time-t'<Th.T)
%           Pert.C.X    = state constraint (e.g. state-x'<Th.X)
%           Pert.P.T    = perturbation corresponding to time constraint
%           Pert.P.X    = perturbation corresponding to state constraint
% OUTPUTS:  P           = probability x control value matrix
%           edges       = grid edges used for binning control variables
%           midpoints   = midpoints of control grid
% NOTES:    N/A
% ISSUES:   Use tensor product packages (TPROD or MTIMESX) around line 42.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% settings
while ~isempty(varargin)
    switch varargin{1}
        case 'nsamp'
            nsamp = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
      varargin(1:2) = [];
end
if ~exist('nsamp','var')
    nsamp = 1000;
end
debug_plots = false;

%% variables
global mdim xdim tsteps;
T = nan(mdim*3,tsteps,nsamp);
U = nan(mdim,tsteps,nsamp);

%% sample trajectory and compute gains
disp('ControlPDF: sampling OFC trajectories...');
for s=1:nsamp
    % sample trajectory
    TX = TrajectorySample(xinit,L,K,A,B,C,H,O,sigma,varargin{:});
    T(1:6,:,s) = TX(1:6,:);
    
    % record gains
%     for m=1:mdim
%         u = -squeeze(L(m,:,:)).*TX;
%         U(m,:,s) = sum(u,1); % sum over state-space
%     end
%     
    % record gains,
    % looped over timestep, copied from TrajectoryRollOut
    for ti=1:tsteps
        Lt          = squeeze(L(:,:,ti));
        x           = TX(:,ti);
        U(:,ti,s)    = -Lt*x;    
    end
end



%% compute state probability density
% Tflat = reshape(T,mdim*3,[]);
% [NS, edges, midpoints, ~] = histcn(Tflat');
% pS  = (NS+eps)/sum(NS(:)+eps);

%% eliminate outliers at "stitch", temp solution
U(U<-50) = nan;
U(U>50) = nan;

%% compute control probability density
Uflat = reshape(U,mdim,[]);
[NU, edges, midpoints, ~] = histcn(Uflat');
pU  = (NU+eps)/sum(NU(:)+eps);

%% debug plots
if debug_plots
    figure(); hold on;
    for m=1:mdim
        for t=1:tsteps
            mu(m,t) = mean(U(m,t,:));
        end
        plot(1:tsteps,mu(m,:));
    end
end

end