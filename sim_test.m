%% FUNCTION: OFC test simulations.
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

% define state-based perturbation
global xinit xdim;
Pert.C.X            = nan(xdim,1);
Pert.Th.X(1)        = 0.5;
Pert.C.X(2,1)       = 2.0;              % if norm(y,Pert.C.X(2,1)) < Pert.Th.X(1), apply Pert.P.X(3)
Pert.P.X.pulse      = zeros(xdim,1);
Pert.P.X.pulse(3)   = 20.0;
%Pert.C.X = [];

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

%% compute OFC trajectories
h = figure();
% chunked
[pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
[TX,QX]    = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q,'Perturbations',Pert);
subplot(1,2,1); hold on;
OFC_SubPlot(TX,trajVar);
title(sprintf('%.1fV | EC=%.1e',Pert.P.X.pulse(3),V),'FontSize',14);

% elemental
[pi,Kpi,V] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
[TX,QX]    = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q,'Perturbations',Pert);
subplot(1,2,2); hold on;
OFC_SubPlot(TX,trajVar);
title(sprintf('%.1fV | EC=%.1e',Pert.P.X.pulse(3),V),'FontSize',14);

suplabel('Chunked | Elemental','x');
suptitle(sprintf('Test - Perturbed OFC Trajectories'));

%% save
savefig(h,'sim_test.fig');
