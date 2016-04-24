%% FUNCTION: OFC-LQGSDN testing temporal adjustments for different initial states.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
clear all; close all; clc;
pgoal1        = [0 10];
pgoal2_1      = [0 10; -5 20];
pgoal2_2      = [0 10; -5 10];
pgoal2_3      = [0 10; -5 0];
pgoal3_1      = [0 10; -5 20; -5 10];
pgoal3_2      = [0 10; -5 10; -5 0];
pgoal3_3      = [0 10; -5 0; -10 0];
pgoals2       = {pgoal2_1,pgoal2_2,pgoal2_3};
pgoals3       = {pgoal3_1,pgoal3_2,pgoal3_3};
Chunks2     = [1 1];
Chunks3     = [1 1 0; 0 0 1];
Chunks      = Chunks2;
Tgoal1Range = 0.2:0.01:0.7; % range of Tgoal(1) to test
Tgoal2Range = 0.5:0.01:1.0; % range of Tgoal(2) to test
Tgoal3Range = 1.0:0.01:1.0;  % range of Tgoal(3) to test
params = OFC_Parameters('pgoal',pgoal2_1);
OFC_PlotSettings();
global xinit Tgoal xdim pgoal ngoal mdim;

% define different initial states
xinit1 = xinit;
xinit2 = xinit;
xinit2(3) = -10; % perturb init velocity

% define state-based perturbation
Pert.C.X      = nan(xdim,1);
Pert.Th.X(1)  = 1.0;
Pert.C.X(2,1) = 2;               % if norm(y,5)<Pert.Th.X(1)
Pert.P.X      = zeros(xdim,1);
%Pert.P.X(1)   = -1;             % perturb px
Pert.P.X(3)   = -20;             % perturb vx

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();


%% unperturbed initial state
xinit(1:6) = xinit1(1:6);
% pgoal2_1
OFC_SetPGOAL(pgoal2_1);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(1,1,ti) = VC;
    VTE(1,1,ti) = VE;
end

% pgoal2_2
OFC_SetPGOAL(pgoal2_2);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(1,2,ti) = VC;
    VTE(1,2,ti) = VE;
end
% pgoal2_3
OFC_SetPGOAL(pgoal2_3);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(1,3,ti) = VC;
    VTE(1,3,ti) = VE;
end


%% perturbed (t=0) initial state
xinit(1:6) = xinit2(1:6);
% pgoal2_1
OFC_SetPGOAL(pgoal2_1);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(2,1,ti) = VC;
    VTE(2,1,ti) = VE;
end

% pgoal2_2
OFC_SetPGOAL(pgoal2_2);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(2,2,ti) = VC;
    VTE(2,2,ti) = VE;
end
% pgoal2_3
OFC_SetPGOAL(pgoal2_3);
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    VTC(2,3,ti) = VC;
    VTE(2,3,ti) = VE;
end


%% plot pgoal x E/C for opt tstep trajectories
% unperturbed
xinit(1:6) = xinit1(1:6);
figure();
for pg=1:3
    OFC_SetPGOAL(pgoals2{pg});
    % elemental
    subplot(2,3,pg); hold on;
    tiOpt         = argmin(VTE(1,pg,:));            % VTE/VTC = xinit x pgoal x tstep
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Target 1 @ %.2fs, E[cost] = %.2f',Tgoal(1),V));
    
    % chunked
    subplot(2,3,3+pg); hold on;
    tiOpt         = argmin(VTC(1,pg,:));
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Target 1 @ %.2fs, E[cost] = %.2f',Tgoal(1),V));
end
suptitle('OT | No Perturbation');

% perturbed
xinit(1:6) = xinit2(1:6);
figure();
for pg=1:3
    OFC_SetPGOAL(pgoals2{pg});
    % elemental
    subplot(2,3,pg); hold on;
    tiOpt         = argmin(VTE(2,pg,:));            % VTE/VTC = xinit x pgoal x tstep
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Target 1 @ %.2fs, E[cost] = %.2f',Tgoal(1),V));
    
    % chunked
    subplot(2,3,pg+3); hold on;
    tiOpt         = argmin(VTC(2,pg,:));
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Target 1 @ %.2fs, E[cost] = %.2f',Tgoal(1),V));
end
suptitle('OT | Perturbation (Time = 0)');


%% plot expected costs as a function of pgoal x E/C
% VTE/VTC = xinit x pgoal x tstep
figure();
for pg=1:3
    subplot(2,3,pg); hold on;
    plot(Tgoal1Range,squeeze(VTC(1,pg,:)),'.-');
    plot(Tgoal1Range,squeeze(VTC(2,pg,:)),'.-');
    legend('Unperturbed','Perturbed');
    title(sprintf('Chunked | Setup %i',pg));
    xlabel('Target 1 time (s)');
    ylabel('Expected cost');
end

for pg=1:3
    subplot(2,3,pg+3); hold on;
    plot(Tgoal1Range,squeeze(VTE(1,pg,:)),'.-');
    plot(Tgoal1Range,squeeze(VTE(2,pg,:)),'.-');
    legend('Unperturbed','Perturbed');
    title(sprintf('Elemental | Setup %i',pg));
    xlabel('Target 1 time (s)');
    ylabel('Expected cost');
end
suptitle('Expected cost as a function of target 1 time');



%% perturbed (t>0) state
xinit(1:6) = xinit1(1:6); % unperturbed initial state
for pg=1:3
    OFC_SetPGOAL(pgoals2{pg});
    for ti=1:numel(Tgoal1Range)
        Tg1 = Tgoal1Range(ti);
        % chunked - adjusted timing
        Tgoal(1) = Tg1;
        [piCAdj,KpiCAdj] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
        % chunked - optimal timing
        tiOpt           = argmin(VTC(1,1,:));            % VTE/VTC = xinit x pgoal x tstep
        Tgoal(1)         = Tgoal1Range(tiOpt);
        [piCOpt,KpiCOpt] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
        % identify perturbation timepoint for policy stitching
        TX = OFC_RollOut(xinit,piCOpt,KpiCOpt,A,B,H,R,Q);
        tiStitch    = argmin(abs(TX(2,1:20) - Pert.C.X(2,1)));
        [piC,KpiC]  = OFC_StitchPolicies(tiStitch,piCOpt,KpiCOpt,piCAdj,KpiCAdj);
        [TXC,QXC]   = OFC_RollOut(xinit,piC,KpiC,A,B,H,R,Q,'Perturbations',Pert);
        QC(pg,ti)    = nansum(QXC);
        
        Tg1 = Tgoal1Range(ti);
        % elemental - adjusted timing
        Tgoal(1) = Tg1;
        [piEAdj,KpiEAdj] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
        % elemental - optimal timing
        tiOpt           = argmin(VTE(1,1,:));            % VTE/VTC = xinit x pgoal x tstep
        Tgoal(1)         = Tgoal1Range(tiOpt);
        [piEOpt,KpiEOpt] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
        % identify perturbation timepoint for policy stitching
        TX = OFC_RollOut(xinit,piEOpt,KpiEOpt,A,B,H,R,Q);
        tiStitch    = argmin(abs(TX(2,1:20) - Pert.C.X(2,1)));
        [piE,KpiE]  = OFC_StitchPolicies(tiStitch,piEOpt,KpiEOpt,piEAdj,KpiEAdj);
        [TXE,QXE]   = OFC_RollOut(xinit,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
        QE(pg,ti)    = nansum(QXE);
        
    end
end

%% plot pgoal x E/C for opt tstep trajectories
xinit(1:6) = xinit1(1:6); % unperturbed initial state
figure();
for pg=1:3
    OFC_SetPGOAL(pgoals2{pg});
    % chunked - optimal adjusted timing
    tiAdj               = argmin(QC(pg,:));
    Tgoal(1)            = Tgoal1Range(tiAdj);
    [piCAdj,KpiCAdj]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    % chunked - optimal timing
    tiOpt               = argmin(VTC(1,1,:));            % VTE/VTC = xinit x pgoal x tstep
    Tgoal(1)            = Tgoal1Range(tiOpt);
    [piCOpt,KpiCOpt]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    % identify perturbation timepoint for policy stitching
    TX                  = OFC_RollOut(xinit,piCOpt,KpiCOpt,A,B,H,R,Q);
    tiStitch            = argmin(abs(TX(2,1:20) - Pert.C.X(2,1)));
    [piC,KpiC]          = OFC_StitchPolicies(tiStitch,piCOpt,KpiCOpt,piCAdj,KpiCAdj);
    % rollout best stitched controller
    [TXC,QXC]           = OFC_RollOut(xinit,piC,KpiC,A,B,H,R,Q,'Perturbations',Pert);
    %plot
    subplot(2,3,pg+3); hold on;
    OFC_SubPlot(TXC,'Position');
    title(sprintf('Target 1 @ %.2fs, Sum(cost) = %.2f',Tgoal1Range(tiAdj),sum(QXC)));
    
    
    % elemental - optimal adjusted timing
    tiAdj               = argmin(QE(pg,:));
    Tgoal(1)            = Tgoal1Range(tiAdj);
    [piEAdj,KpiEAdj] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    % elemental - optimal timing
    tiOpt               = argmin(VTE(1,1,:));            % VTE/VTC = xinit x pgoal x tstep
    Tgoal(1)            = Tgoal1Range(tiOpt);
    [piEOpt,KpiEOpt]    = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    % identify perturbation timepoint for policy stitching
    TX                  = OFC_RollOut(xinit,piEOpt,KpiEOpt,A,B,H,R,Q);
    tiStitch            = argmin(abs(TX(2,1:20) - Pert.C.X(2,1)));
    [piE,KpiE]          = OFC_StitchPolicies(tiStitch,piEOpt,KpiEOpt,piEAdj,KpiEAdj);
    % rollout best stitched controller
    [TXE,QXE]           = OFC_RollOut(xinit,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);
    %plot
    subplot(2,3,pg); hold on;
    OFC_SubPlot(TXE,'Position');
    title(sprintf('Target 1 @ %.2fs, Sum(cost) = %.2f',Tgoal1Range(tiAdj),sum(QXE)));
end
suptitle('OT | Perturbation (Time > 0)');

