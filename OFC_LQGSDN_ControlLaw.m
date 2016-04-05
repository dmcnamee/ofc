function [L,Sx1,Se1,s1] = OFC_LQGSDN_ControlLaw(K,A,B,C,H,O,R,Q)
%% FUNCTION: OFC LQGSDN control law given Kalman filter.
% INPUTS:   K = Kalman filter
%           A = state-evolution matrix
%           B = control-input matrix
%           C = signal-dependent noise matrix
%           H = sensory-indicator matrix
%           O = sensory noise covariance
%           R = energy-cost matrix
%           Q = goal-targets-cost matrix (assumes via-points transversed at regular intervals)
% OUTPUTS:  L   = time-varying feedback control gains (control law)
%           Sx1 = component of cost-to-go at first tstep due to state
%           Se1 = component of cost-to-go at first tstep due to estimation error
%           s1  = component of cost-to-go at first tstep due to control/state
% NOTES:    State evolution noise (Eqn 3.1, Todorov2005), 
%           internal noise (p1090, Todorov2005) 
%           and state-dependent noise (Eqn 3.1, Todorov2005) are ignored.
%           Thus, corresponds to fully observable case (Eqn 4.3, Todorov2005)
% ISSUES:   N/A
% REFS:     Todorov2002 (SI, Eqn 3) / Todorov2005
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% global variables
global tsteps ngoal xdim mdim T;

%% setup variables
Sxt = squeeze(Q(ngoal,:,:));                    % component of cost-to-go due to state (init. to final goal-target)
Set = zeros(xdim,xdim);                         % component of cost-to-go due to estimation error (init. to zero)
st  = 0;                                        % component of cost-to-go due to independent noise sources
L   = nan(tsteps,mdim,xdim);

%% backward-time-sweep calculation
for ti=fliplr(1:tsteps)
    t  = T(ti);                                 % time
    Kt = squeeze(K(ti,:,:));                    % corresponding Kalman gain
    Lt = B'*Sxt*B + R;                          % update feedback gain
    for i=1:2
        Ci = squeeze(C(i,:,:));
        Lt = Lt + Ci'*(Sxt+Set)*Ci;
    end
    Lt        = (Lt^-1)*B'*Sxt*A;
    L(ti,:,:) = Lt;
    Qt  = OFC_TaskConstraintCostMatrix(t,Q);    % update cost-to-go matrices
    Sxt = Qt + A'*Sxt*(A-B*Lt);
    Set = A'*Sxt*B*Lt + (A-Kt*H)'*Set*(A-Kt*H);
    st  = trace(Set*Kt*O*Kt');                  % ignoring control/state-indepenent noise components here
    
    if any(isnan(Lt(:)))
        error('OFC_LQGSDN_ControlLaw: nan returned.');
    end
end

%% output cost-to-go components
Sx1 = Sxt;
Se1 = Set;
s1  = st;

end
