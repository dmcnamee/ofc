function [K,Gx1,Ge1] = OFC_LQGSDN_KalmanFilter(x,L,A,B,C,H,O)
%% FUNCTION: OFC LQGSDN Kalman filter.
% INPUTS:   x   = initial state of system
%           L   = control law
%           A   = state-evolution matrix
%           B   = control-input matrix
%           C   = signal-dependent noise matrix
%           H   = sensory-indicator matrix
%           O   = sensory-noise-covariance matrix
% OUTPUTS:  K   = time-varying Kalman gains (Kalman filter)
%           Gx1 = non-centered covariance of the state estimate at first tstep
%           Ge1 = expected estimation error covariance at first tstep
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002 (SI, Eqn 2)
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% global variables
global tsteps xdim;

%% setup variables
Ge = zeros(xdim,xdim);                      % expected estimation error covariance (initialized to zero covariance)
Gx = x*x';                                  % non-centered covariance of state estimation
K  = nan(xdim,xdim,tsteps);

%% forward-time-sweep calculation
for ti=1:tsteps
    Lt = squeeze(L(:,:,ti));                % corresponding feedback control gain
    Kt        = A*Ge*H'*((H*Ge*H' + O)^-1); % update Kalman gain
    K(:,:,ti) = Kt;
    Ge = (A-Kt*H)*Ge*A';                    % update estimation covariance update
    for i=1:2
        Ci = squeeze(C(i,:,:));
        Ge = Ge + Ci*Lt*Gx*Lt'*Ci';
    end
    Gx = Kt*H*Ge*A' + (A-B*Lt)*Gx*(A-B*Lt)';
    
    if any(isnan(Kt(:)))
        error('OFC_LQGSDN_KalmanFilter: nan returned.');
    end
end

%% output error variance components
Gx1 = x*x';
Ge1 = zeros(xdim,xdim); % initialized to zero covariance

end
