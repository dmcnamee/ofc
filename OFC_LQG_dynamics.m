function [A,B,C] = OFC_LQG_dynamics()
%% FUNCTION: OFC dynamics of LQG model with no signal-dependent noise (state-space formalism).
% INPUTS:   N/A
% OUTPUTS:  A = state-evolution matrix
%           B = control-input matrix
%           C = signal-dependent noise matrix
% NOTES:    N/A
% REFS:     Todorov2002* / Todorov2005 / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
% EDITED:   Hannah Sheahan, sheahan.hannah@gmail.com, (Oct-2017)

%% global variables
global xdim tres smdsteps m tau mdim ngoal c1 c2;

%% state-evolution
A       = eye(xdim,xdim);
A(1,3)  = tres; A(2,4) = tres;                          % state-update
A(3,5)  = tres/m; A(4,6) = tres/m;                      % velocity-update
A(5,5)  = 1-(tres/tau); A(6,6) = 1-(tres/tau);            % actuator-evolution

if smdsteps ~= 0                                        % incorporate sensorimotor delay
    Asmd    = zeros(xdim,xdim);
    stateI  = 1:3*mdim;
    Asmd(stateI,stateI) = A(stateI,stateI);             % update most recent state
    goalI = (1:(ngoal+1)*mdim) + (smdsteps+1)*3*mdim;   
    Asmd(goalI,goalI)   = A(goalI,goalI);               % propagate goal-position information
    for d=1:smdsteps
        Iold            = (1:3*mdim) + (d-1)*3*mdim;
        Inew            = (1:3*mdim) + d*3*mdim;
        Asmd(Inew,Iold) = eye(3*mdim,3*mdim);           % shift-down operator
    end
    A = Asmd;
end

%% control-input
B       = zeros(xdim,mdim);
B(5,1)  = (tres/tau); B(6,2) = (tres/tau); % control-signal input to actuator update

%% signal-dependent noise (adapted from [Liu2007])
C       = zeros(2,mdim,mdim);
C(1,1,1) = c1; C(1,2,2) = c1; C(1,:,:) = B*squeeze(C(1,:,:));
C(2,2,1) = -c2; C(2,1,2) = c2; C(2,:,:) = B*squeeze(C(2,:,:));

end
