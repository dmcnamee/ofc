function [pi,Kpi,V] = OFC_LQGSDN(x,A,B,C,H,O,R,Q)
%% FUNCTION: OFC LQGSDN solution based on iterative Kalman filter/ control law optimization.
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
% ISSUES:   Use structures for dynamics matrices etc.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global tsteps mdim xdim optThresh initDiff maxIter;
L = zeros(mdim,xdim,tsteps);
K = zeros(xdim,xdim,tsteps);
Knew = zeros(mdim,xdim,tsteps); Lnew = zeros(mdim,xdim,tsteps);
iter = 0;
diff = initDiff;

%% iterate
tic;
% figure(); hold on;
% xlabel('Iteration $i$'); ylabel('Change in control law $L$, $|| L(i+1) - L(i) ||^2$');
while (diff(end) > optThresh) && (iter < maxIter)
    iter = iter + 1;
    %fprintf('OFC_LQGSDN: diff(L) = %.3f, iteration %i...',diff,iter);
    % Kalman filter
    [K,Gx1,Ge1]    = OFC_LQGSDN_KalmanFilter(x,L,A,B,C,H,O);
    % Optimal control law
    [L,Sx1,Se1,s1] = OFC_LQGSDN_ControlLaw(K,A,B,C,H,O,R,Q);
    % update L,K
    Kold = Knew; Knew = K;
    Lold = Lnew; Lnew = L;
    diff(iter) = norm(Lold(:)-Lnew(:),2);
    % plot
    % plot(1:iter,diff);
end

%% optimal controller
pi  = L;
Kpi = K;
V   = OFC_LQGSDN_CostToGo(x,Gx1,Ge1,Sx1,Se1,s1);

%% diagnostics
%disp(Lold); disp(Lnew);
fprintf('OFC_LQGSDN: '); toc;
fprintf('OFC_LQGSDN: #iterations until convergence = %i, with diff(L) = %.3f.\n',iter,diff(end));
% fprintf('OFC_LQFSDN: expected cost-to-go = %.3f.\n',V);

end
