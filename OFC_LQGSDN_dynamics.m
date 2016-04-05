function [A,B,C] = OFC_LQGSDN_dynamics()
%% FUNCTION: OFC dynamics of LQGSDN model (state-space formalism).
% INPUTS:   N/A
% OUTPUTS:  A = state-evolution matrix
%           B = control-input matrix
%           C = signal-dependent noise matrix
% NOTES:    N/A
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% global variables
global xdim tres m tf mdim c1 c2;

%% state-evolution
A       = eye(xdim,xdim);
A(1,3)  = tres; A(2,4) = tres;                   % state-update
A(3,5)  = tres/m; A(4,6) = tres/m;               % velocity-update
A(5,5)  = exp(-tres/tf); A(6,6) = exp(-tres/tf); % actuator-evolution

%% control-input
B       = zeros(xdim,mdim);
B(5,1)  = 1; B(6,2) = 1;                         % control-signal input to actuator update

%% signal-dependent noise (adapted from [Liu2007])
C       = zeros(2,xdim,mdim);
C(1,:,:) = B*[c1 0; 0 c1];
C(2,:,:) = B*[0 c2; -c2 0];


end
