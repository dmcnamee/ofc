%% FUNCTION: Example optimal feedback control simulations of movements through via-points
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     N/A
% AUTHORS:   Daniel McNamee, daniel.c.mcnamee@gmail.com; 
%            Hannah Sheahan, sheahan.hannah@gmail.com

% In these examples, no perturbations were applied to the movements. We
% consider two distinct simulations, one which considers the first of the
% two targets as a via-point (chunked), and the other which considers each
% of the two targets as distinct goals under different optimisations 
% (and a shorter planning horizon).

%%
clear all; close all; clc;

%% Chunked movement settings
pvia        = [0 12];
psecond     = [7.0711, 19.0711];
pgoal       = [pvia; psecond];
smdelay     = 0.0;
params      = OFC_Parameters('pgoal',pgoal,'smdelay',smdelay); % defaults to tmax=0.8, and tvia = 0.4;
Chunks      = [1 1];   % single cohesive movement
OFC_PlotSettings();
trajVar = 'Position'; % trajectory variable to plot (e.g. 'Velocity')
global xinit;

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% Compute chunked OFC trajectories
[pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);        % no force-field

figure();
subplot(1,2,1); hold on;
[TX,QX]    = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q); % apply OFC gains from no forcefield movements
OFC_SubPlot(TX,trajVar);
title('Cohesive')

%% Segmented movement settings
clear global *
clearvars -except nsamp;
pvia        = [0 12];
psecond     = [7.0711, 19.0711];
pgoal       = [pvia; psecond];
smdelay     = 0.0;
params      = OFC_Parameters('pgoal',pgoal,'smdelay',smdelay); % defaults to tmax=0.8, and tvia = 0.4;
Chunks      = [1 0; 0 1];   % segmented movements
OFC_PlotSettings();
trajVar = 'Position'; % trajectory variable to plot (e.g. 'Velocity')
global xinit;

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% Compute segmented OFC trajectories
[pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);        % no force-field
D = motor_description_length(pi); 

subplot(1,2,2); hold on;
[TX,QX]    = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q); % apply OFC gains from no forcefield movements
OFC_SubPlot(TX,trajVar);
title('Segmented')

%%
