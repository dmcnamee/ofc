function TX = TrajectorySample(x,L,K,A,B,C,H,O,sigma,varargin)
%% FUNCTION: Samples trajectory TX WITH NOISE under control law L and sensory filter K.
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
% OUTPUTS:  TX          = time x state matrix
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% settings
time_buffer = 0.2; % percentage of trials to omit noise (end of movement)

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
global tsteps mdim xdim xinit;
TX = zeros(xdim,tsteps);    % discretized time/state-space
if numel(x)==mdim           % extend to include target positions
    x = [x xinit(3*mdim+1:xdim)]';
end

%% init time/state-space
TX(:,1)        = x;
if exist('Pert','var')
    [TX(:,1),Pert] = OFC_ApplyPerturbation(1,TX(:,1),Pert);         % apply perturbation
end

TXnans = true; % sample trajectories without nans
while TXnans
    for ti=2:tsteps
        Lt          = squeeze(L(:,:,ti));                               % corresponding feedback control gain
        x           = TX(:,ti-1);                                       % set current state
        u           = -Lt*x;                                            % derive control input from control law
        if tsteps-ti>tsteps*time_buffer
            noise(ti)   = normrnd(0,sigma);                                 % sample of noise
        else
            % don't inject noise toward end of movement (Liu 2007)
            noise(ti) = 0;
        end
        TX(:,ti)    = A*x + B*u + noise(ti)*sum(sum(u.*C.*u,1),3)';     % state evolution (no noise), signal-dependent noise
        if exist('Pert','var')
            [TX(:,ti),Pert] = OFC_ApplyPerturbation(ti,TX(:,ti),Pert);  % apply tstep-state perturbation
        end
    end
    if any(isnan(TX(:)))
        TXnans = true;
%         disp('TrajectorySample: nan returned.');
    else
        TXnans = false;
    end  
end

%% plot
if plot
    OFC_Plot(TX,'L',L,varargin{:});
end

end
