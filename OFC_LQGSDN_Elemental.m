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
global ngoal;
Chunks  = eye(ngoal,ngoal);

%% special case of chunked solution
[pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);

end
