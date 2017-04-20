%% FUNCTION: Test motor_description_length.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% settings
clear all; close all; clc;
pgoal       = [0 7; 7 14];
smdelay     = 0.05;
params      = OFC_Parameters('pgoal',pgoal,'smdelay',smdelay);
Chunks      = [1 1];
OFC_PlotSettings();
trajVar = 'Position'; % trajectory variable to plot (e.g. 'Velocity')
global xinit;

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% include clockwise velocity-dependent force (I think!)
AF = A;
k = 13; % Howard/Wolpert/Franklin2013
AF(5,4) = -k;
AF(6,3) = k;

%% compute OFC trajectories (assume chunked)
[pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);        % no force-field
[piF,KpiF,VF] = OFC_LQGSDN_Chunked(Chunks,xinit,AF,B,C,H,O,R,Q);    % force-field
D = motor_description_length(pi); 
DF = motor_description_length(piF);

figure();
subplot(1,2,1); hold on;
[TX,QX]    = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q); % apply OFC gains from no forcefield movements
OFC_SubPlot(TX,trajVar);

subplot(1,2,2); hold on;
[TX,QX]    = OFC_RollOut(xinit,piF,KpiF,AF,B,H,R,Q); % apply OFC gains from forcefield movements
OFC_SubPlot(TX,trajVar);
DF>D



