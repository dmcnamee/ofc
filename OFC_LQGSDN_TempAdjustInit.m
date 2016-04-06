%% FUNCTION: OFC-LQGSDN testing temporal adjustments for different initial states.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
clear all; close all; clc;
pgoal1      = [0 10];
pgoal2_1      = [0 10; -5 20];
pgoal2_2      = [0 10; -5 10];
pgoal2_3      = [0 10; -5 0];
% pgoal3      = [0 10; -5 20; -5 10];
% pgoal3      = [0 10; -5 10; -5 0];
pgoal3      = [0 10; -5 0; -10 0];
Chunks2     = [1 1];
Chunks3     = [1 1 0; 0 0 1];
Chunks      = Chunks2;
Tgoal1Range = 0.3:0.001:0.7; % range of Tgoal(1) to test
Tgoal2Range = 0.55:0.02:1.15; % range of Tgoal(2) to test
Tgoal3Range = 1.0:0.02:1.0;  % range of Tgoal(3) to test
params = OFC_Parameters('pgoal',pgoal2_1);
OFC_PlotSettings();
global xinit Tgoal pgoal ngoal mdim;

% define different initial states
xinit1 = xinit;
xinit2 = xinit;
xinit2(3) = -10; % does not seem to be much effect of init velocity

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

%% unperturbed initial state
% pgoal2_1
xinit = xinit1;
pgoal = pgoal2_1;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(1,1,ti) = VC;
    QTE(1,1,ti) = VE;
end

% pgoal2_2
pgoal = pgoal2_2;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(1,2,ti) = VC;
    QTE(1,2,ti) = VE;
end
% pgoal2_3
pgoal = pgoal2_3;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(1,3,ti) = VC;
    QTE(1,3,ti) = VE;
end


%% perturbed initial state
% pgoal2_1
xinit = xinit2;
pgoal = pgoal2_1;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(2,1,ti) = VC;
    QTE(2,1,ti) = VE;
end

% pgoal2_2
pgoal = pgoal2_2;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(2,2,ti) = VC;
    QTE(2,2,ti) = VE;
end
% pgoal2_3
pgoal = pgoal2_3;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTC(2,3,ti) = VC;
    QTE(2,3,ti) = VE;
end


%% plot pgoal x E/C for opt tstep trajectories
Chunks = Chunks2;
pgoals = {pgoal2_1,pgoal2_2,pgoal2_3};
% unperturbed
figure();
for pg=1:3
    pgoal         = pgoals{pg};
    xinit         = [xinit1(1:6)' reshape(pgoal',1,ngoal*mdim)]';
    % elemental
    subplot(3,2,(pg-1)*2+1); hold on;
    tiOpt         = argmin(QTE(1,pg,:));            % QTE/QTC = xinit x pgoal x tstep
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Elemental | Setup %i | Target 1 @ %.2fs, E[cost] = %.2f',pg,Tgoal(1),V));
    
    % chunked
    subplot(3,2,pg*2); hold on;
    tiOpt         = argmin(QTC(1,pg,:));
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Chunked | Setup %i | Target 1 @ %.2fs, E[cost] = %.2f',pg,Tgoal(1),V));
end
suptitle('OT | Unperturbed Initial State');

% perturbed
figure();
for pg=1:3
    pgoal         = pgoals{pg};
    xinit         = [xinit2(1:6)' reshape(pgoal',1,ngoal*mdim)]';
    % elemental
    subplot(3,2,(pg-1)*2+1); hold on;
    tiOpt         = argmin(QTE(2,pg,:));            % QTE/QTC = xinit x pgoal x tstep
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Elemental | Setup %i | Target 1 @ %.2fs, E[cost] = %.2f',pg,Tgoal(1),V));
    
    % chunked
    subplot(3,2,pg*2); hold on;
    tiOpt         = argmin(QTC(2,pg,:));
    Tgoal(1)      = Tgoal1Range(tiOpt);
    [pi,Kpi,V]    = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    [TX,QX]       = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
    OFC_SubPlot(TX,'Position');
    title(sprintf('Chunked | Setup %i | Target 1 @ %.2fs, E[cost] = %.2f',pg,Tgoal(1),V));
end
suptitle('OT | Perturbed Initial State');
