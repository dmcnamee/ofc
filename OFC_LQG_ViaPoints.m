%% FUNCTION: OFC LQG model without control-signal-dependent noise.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    Unfinished.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
% TOOLBOXES:    Control Systems Toolbox (http://uk.mathworks.com/products/control/)

%% set global parameters
clear all; close all; clc;
global mdim;
global plim vlim ulim;
global tres pres vres ures;
global pgoal goalsize ngoal;
global b tau m tf;
global tsteps psteps vsteps usteps;
global c1 c2 usigma;
global Wenergy Wtime Wstop Wgoal Wtimeout;
global tmax vmax;
varargin = {};
ofc_parameters(varargin{:});


%% setup variables
xinit = [0 0 0 0 0 0]'; % initial position-velocity-actuator state of the system
for n=1:ngoal
    xinit = [xinit' pgoal(n,:)]'; % add goal-target positions to state definition
end

%% discretize time-space
T           = 0:tmax:tres;                                              % time space


%% LQG solution
[A,B,C] = OFC_LQG_dynamics();
[H,O] = OFC_LQG_feedback();
[R,Q] = OFC_LQG_costfunc();
L = OFC_LQG(xinit,A,B,C,H,O,R,Q);
