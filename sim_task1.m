%% FUNCTION: OFC simulations for Task 1 - learning motor Chunks and 
%%           solving perturbations for 2-vpoints at varying angles.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    Only optimizing over first via-point timing.
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% settings
clear all; close all; clc;
% polar grid
r       = 7;
angles  = 0:pi/4:7/4*pi;
for g=1:numel(angles)                   % degrees
    phi         = angles(g);
    pgoals{g}   = [0 7; r*cos(phi) 7+r*sin(phi)];
end
% % cartesian grid
% pgoals      =  {[0 7; 0 14],...
%                 [0 7; 7 14],...
%                 [0 7; 7 7],...
%                 [0 7; 7 0],...
%                 [0 7; 0 0],...
%                 [0 7; -7 0],...
%                 [0 7; -7 7],...
%                 [0 7; -7 14]};
params      = OFC_Parameters('pgoal',pgoals{1});
Chunks      = [1 1];
pertTime    = 0.2;                  % s, time over which perturbation applied
sinInt      = 2;                    % -(cos(pi) - cos(0))
perts       = [-100 -50 0 50 100];  % velocity changes resulting from step force
% (task code distributes force impulse over sin wave 0-180degrees which integrates to scale x2)

% timing optimization
TMod        = 1;                    % optimize over timing through first vpoint
TgoalRes    = 0.01;                 % resolution of Tgoal range to compute over
TgoalMin    = [0.2 0.5];
TgoalMax    = [0.7 1.0];

OFC_PlotSettings();
trajVar = 'Position'; % trajectory variable to plot (e.g. 'Velocity')
global xinit xdim plim;
plim = [-20 20; -5 20];

% define state-based perturbation
Pert.C.X        = nan(xdim,1);
Pert.Th.X(1)    = 0.5;
Pert.C.X(2,1)   = 3.0;             % if norm(y,Pert.C.X(2,1)) < Pert.Th.X(1), apply Pert.P.X(3)
Pert.P.X.pulse  = zeros(xdim,1);

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

%% variables
np = numel(perts);      % number of perturbations
nm = numel(pgoals);     % number of distinct movements
ng = size(pgoals{1},1); % number of vpoint goals
for g=1:ng
    TgoalRange{g} = TgoalMin(g):TgoalRes:TgoalMax(g);         % s, range of Tgoal to test
end

%% plot OFC trajectories (figure = vpoint config, interleaved rows of elemental/chunked solution, col iterates over perturbations)
h.mainfig   = figure();
h.tabgroup  = uitabgroup(h.mainfig);
for m=1:nm                              % figure iteration
    h.tab(m) = uitab(h.tabgroup, 'Title', sprintf('Config %i', m));
    axes('Parent', h.tab(m));

    OFC_SetPGOAL(pgoals{m});            % set viapoints
    for p=1:np                          % col iteration
        Pert.P.X.pulse(3) = perts(p);   % set perturbation x-velocity
        % elemental
        [piOptE,KpiOptE,TOptE]  = OFC_LQGSDN_OptimizeTiming(TMod,TgoalRange{TMod},A,B,C,H,O,R,Q,'Perturbations',Pert);
        [TXE,QXE]               = OFC_RollOut(xinit,piOptE,KpiOptE,A,B,H,R,Q,'Perturbations',Pert);
        subplot(2,np,p); hold on;       %plot
        OFC_SubPlot(TXE,trajVar);
        title(sprintf('%.1fV | @%.2fs | C=%.1e',perts(p),TOptE,sum(QXE)),'FontSize',13);
        
        % chunked
        [piOptC,KpiOptC,TOptC]  = OFC_LQGSDN_OptimizeTiming(TMod,TgoalRange{TMod},A,B,C,H,O,R,Q,'Chunks',Chunks,'Perturbations',Pert);
        [TXC,QXC]               = OFC_RollOut(xinit,piOptC,KpiOptC,A,B,H,R,Q,'Perturbations',Pert);
        subplot(2,np,np + p); hold on;  % plot
        OFC_SubPlot(TXC,trajVar);
        title(sprintf('%.1fV | @%.2fs | C=%.1e',perts(p),TOptC,sum(QXC)),'FontSize',13);
    end
%     suplabel('Perturbations','x');
%     suplabel('Chunked / Elemental Solution','y');
%     suptitle(sprintf('Task 1 - OFC Trajectories - VPoint Config %i',m));
end

%% save
savefig(h.mainfig,'sim_task1.fig');
