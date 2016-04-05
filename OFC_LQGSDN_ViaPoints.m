%% FUNCTION: OFC LQGSDN ViaPoints model with control-signal-dependent noise.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

clear all; close all; clc;

%% variables
OFC_Parameters();
pgoal1      = [0 10]';
pgoal2      = [0 10; -5 20]';
pgoal3      = [0 10; -5 20; -5 10]';
% pgoal3      = [0 10; -5 10; -5 0]';
% pgoal3      = [0 10; -5 0; -10 0]';
OFC_Parameters('pgoal',pgoal3);
TgoalAdj    = [0.4 0.76 1.0];
global Tgoal; TgoalDef = Tgoal;

%% Perturbations
% % define time-based perturbation
% global xdim tsteps Tgoal;
% Pert.TX          = zeros(xdim,tsteps);
% tpert            = time2step(Tgoal(1)/2);
% Pert.TX(1,tpert) = -2;

% define state-based perturbation
global xdim;
Pert.C.X      = nan(xdim,1);
Pert.Th.X(1)  = 0.6;
Pert.C.X(1,2) = 1;               % if norm(y,5)<pres
Pert.P.X      = zeros(xdim,1);
%Pert.P.X(1)   = -1;             % perturb px->-1
Pert.P.X(3)   = -10;             % perturb vx->-15


% %% Problem 1: one point
% % set global parameters
% OFC_Parameters('pgoal',[0 10]');
% global xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN solution
% [pi,Kpi] = OFC_LQGSDN(xinit,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% [TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
% OFC_Plot(TX);
% 
% 
% %% Problem 2: two points
% % set global parameters
% OFC_Parameters('pgoal',[0 10; -5 20]');
% global xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN solution
% [pi,Kpi] = OFC_LQGSDN(xinit,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
% OFC_Plot(TX);
% 
% 
% %% Problem 3: two points, elemental solution
% % set global parameters
% pgoal = [0 10; -5 20]';
% OFC_Parameters('pgoal',pgoal);
% global xinit;
% x = xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN-Elemental solution
% [piE,KpiE] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
% OFC_Plot(TX);
% 
% 
% %% Problem 4: three points, 3 chunk
% % set global parameters
% OFC_Parameters('pgoal',pgoal3);
% global xinit;
% x = xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN solution
% [piE,KpiE] = OFC_LQGSDN(x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
% OFC_Plot(TX);
% 
% 
%% Problem 5: three points, elemental
% set global parameters
OFC_Parameters('pgoal',pgoal3);
global xinit;
x = xinit;

% setup variables
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% LQGSDN solution
[piE,KpiE] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);

% rollout and plot
[TX,QX] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | No Perturbation | No Temporal Adjustment | Cost = %.2f',sum(QX)));


% %% Problem 6: three points, 1+2 chunks
% % set global parameters
% OFC_Parameters('pgoal',pgoal3);
% global xinit;
% x = xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN-Chunked solution
% Chunks      = [1 0 0; 0 1 1];
% [piE,KpiE]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
% OFC_Plot(TX);


%% Problem 7: three points, 2+1 chunks
% set global parameters
OFC_Parameters('pgoal',pgoal3);
global xinit;
x = xinit;

% setup variables
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% LQGSDN-Chunked solution
Chunks      = [1 1 0; 0 0 1];
[piE,KpiE]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);

% rollout and plot
[TX,QX] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | 2+1 Chunks | No Perturbation | No Temporal Adjustment | Cost = %.2f',sum(QX)));

% 
% %% Problem 8: three points, elemental with perturbation
% % set global parameters
% OFC_Parameters('pgoal',pgoal3);
% global xinit Tgoal pres tsteps mdim xdim T;
% x = xinit;
%
% setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN solution
% [piE,KpiE] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
% OFC_Plot(TX);
% 
% 
% %% Problem 9: three points, 2+1 chunks with perturbation
% % set global parameters
% OFC_Parameters('pgoal',pgoal3);
% global Tgoal; Tgoal = TgoalDef;
% global xinit; x = xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN-Chunked solution
% Chunks      = [1 1 0; 0 0 1];
% [piE,KpiE]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
% OFC_Plot(TX);
% 
% 
% %% Problem 10: three points, 2+1 chunks with perturbation and time delay to first target
% % set global parameters
% OFC_Parameters('pgoal',pgoal3);
% global Tgoal; Tgoal = TgoalAdj;
% global xinit; x = xinit;
% 
% % setup variables
% [A,B,C] = OFC_LQGSDN_dynamics();
% [H,O]   = OFC_LQG_feedback();
% [R,Q]   = OFC_LQG_costfunc();
% 
% % LQGSDN-Chunked solution
% Chunks      = [1 1 0; 0 0 1];
% [piE,KpiE]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);
% 
% % rollout and plot
% TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
% %TX = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
% OFC_Plot(TX);

%% Problem 11: three points, 2+1 chunks with perturbation and Tgoal modification when perturbed
% set global parameters
OFC_Parameters('pgoal',pgoal3);
global xinit; x = xinit;

% setup variables
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% LQGSDN-Chunked "fast" (default,regular intervals) solution
Chunks      = [1 1 0; 0 0 1];
global Tgoal; Tgoal = TgoalDef;
[piCF,KpiCF]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);

% LQGSDN-Chunked "slowed" (perturbed) solution
Chunks      = [1 1 0; 0 0 1];
global Tgoal; Tgoal = TgoalAdj;
[piCS,KpiCS]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);

% stitch control policies together at perturbed tstep = 6
% TX = OFC_RollOut(x,piCF,KpiCF,A,B,H,R,Q,'Perturbations',Pert); % testing perturbation tstep
tiStitch = 6;
[piC,KpiC] = OFC_StitchPolicies(tiStitch,piCF,KpiCF,piCS,KpiCS);

% rollout and plot
global Tgoal; Tgoal = TgoalDef;
[TX,QX]   = OFC_RollOut(x,piCF,KpiCF,A,B,H,R,Q,'Perturbations',Pert);
OFC_Plot(TX,'Description',sprintf('OT | 2+1 Chunks | State-Based Perturbation | No Online Temporal Adjustment | Cost = %.2f',sum(QX)));
global Tgoal; Tgoal = TgoalAdj;
[TX,QX]     = OFC_RollOut(x,piC,KpiC,A,B,H,R,Q,'Perturbations',Pert);
OFC_Plot(TX,'Description',sprintf('OT | 2+1 Chunks | State-Based Perturbation | Online Temporal Adjustment | Cost = %.2f',sum(QX)));


%% Problem 12: three points, elemental with perturbation and Tgoal modification when perturbed
% set global parameters
OFC_Parameters('pgoal',pgoal3);
global xinit; x = xinit;

% Perturbations
% % define time-based perturbation
% global xdim tsteps Tgoal;
% Pert.TX          = zeros(xdim,tsteps);
% tpert            = time2step(Tgoal(1)/2);
% Pert.TX(tpert,1) = -2;

% define state-based perturbation
global xdim;
Pert.C.X      = nan(xdim,1);
Pert.Th.X(1)  = 0.6;
Pert.C.X(1,2) = 1;               % if norm(y,5)<pres
Pert.P.X      = zeros(xdim,1);
%Pert.P.X(1)   = -1;             % perturb px->-1
Pert.P.X(3)   = -10;             % perturb vx->-15

% setup variables
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% LQGSDN-Chunked "fast" (default,regular intervals) solution
[piEF,KpiEF]  = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);

% LQGSDN-Chunked "slowed" (perturbed) solution
Chunks      = [1 1 0; 0 0 1];
global Tgoal; Tgoal = TgoalAdj;
[piES,KpiES]  = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);

% stitch control policies together at perturbed tstep = 6
% TX = OFC_RollOut(x,piCF,KpiCF,A,B,H,R,Q,'Perturbations',Pert); % testing perturbation tstep, tiStitch = 6
tiStitch = 6;
[piE,KpiE] = OFC_StitchPolicies(tiStitch,piEF,KpiEF,piES,KpiES);

% rollout and plot
global Tgoal; Tgoal = TgoalDef;
[TX,QX] = OFC_RollOut(x,piEF,KpiEF,A,B,H,R,Q,'Perturbations',Pert);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | State-Based Perturbation | No Temporal Adjustment | Cost = %.2f',sum(QX)));
global Tgoal; Tgoal = TgoalAdj;
[TX,QX] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | State-Based Perturbation | Online Temporal Adjustment | Cost = %.2f',sum(QX)));
