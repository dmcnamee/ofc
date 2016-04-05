function [H,O] = OFC_LQG_feedback()
%% FUNCTION: OFC feedback in LQG model (state-space formalism).
% INPUTS:   N/A
% OUTPUTS:  H = sensory indicator matrix
%           O = sensory noise covariance matrix ("Omega")
% NOTES:    N/A
% ISSUES:   Include off-diagonal components
% REFS:     Todorov2002* / Todorov2005 / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% global variables
global xdim psigma vsigma asigma ssigma;

%% sensory indicator matrix
H           = zeros(xdim,xdim);
H(1:6,1:6)  = eye(6,6);

%% sensory noise covariance matrix
O          = eye(xdim,xdim);
O(1:2,1:2) = O(1:2,1:2)*psigma; % position noise
O(3:4,3:4) = O(3:4,3:4)*vsigma; % velocity noise
O(5:6,5:6) = O(5:6,5:6)*asigma; % actuator noise
O          = O*ssigma; % overall sensory noise scaling

end
