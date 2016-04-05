%% FUNCTION: OFC-LQGSDN testing temporal adjustments for different initial states.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Chunked not working.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
clear all; close all; clc;
pgoal1      = [0 10];
pgoal2      = [0 10; -5 10];
% pgoal3      = [0 10; -5 20; -5 10];
% pgoal3      = [0 10; -5 10; -5 0];
pgoal3      = [0 10; -5 0; -10 0];
pgoal       = pgoal2;
Chunks2     = [1 1];
Chunks3     = [1 1 0; 0 0 1];
Chunks      = Chunks2;
Tgoal1Range = 0.3:0.01:0.7; % range of Tgoal(1) to test
Tgoal2Range = 0.55:0.02:1.15; % range of Tgoal(2) to test
Tgoal3Range = 1.0:0.02:1.0;  % range of Tgoal(3) to test
params = OFC_Parameters('pgoal',pgoal);
global xinit Tgoal;

% define different initial states
xinit1 = xinit;
xinit2 = xinit;
xinit2(3) = -10; % does not seem to be much effect of init velocity

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% chunked
xinit = xinit1;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    QTC(1,ti) = VC;
end
xinit = xinit2;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piC,KpiC,VC] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    QTC(2,ti) = VC;
end

%elemental
xinit = xinit1;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTE(1,ti) = VE;
end
xinit = xinit2;
for ti=1:numel(Tgoal1Range)
    Tgoal(1) = Tgoal1Range(ti);
    [piE,KpiE,VE] = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    QTE(2,ti) = VE;
end

% QT = tstep x xinit x E/C
T1E = argmin(QTE(1,:));
T1C = argmin(QTC(1,:));
T2E = argmin(QTE(2,:));
T2C = argmin(QTC(2,:));

figure(); 
subplot(1,2,1); hold on;
plot(Tgoal1Range,squeeze(QTC(1,:)),'.-');
plot(Tgoal1Range,squeeze(QTC(2,:)),'.-');
xlabel('Target 1 time (s)'); ylabel('Expected cumulative cost');
legend('xinit1','xinit2');
title('OT | 2 Chunked | Perturbation | Temporal Adjustment');
subplot(1,2,2); hold on;
plot(Tgoal1Range,squeeze(QTE(1,:)),'.-');
plot(Tgoal1Range,squeeze(QTE(2,:)),'.-');
xlabel('Target 1 time (s)'); ylabel('Expected cumulative cost');
legend('xinit1','xinit2');
title('OT | Elemental | Perturbation | Temporal Adjustment');

% rollout and plot
Tg1 = 0.1; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));

Tg1 = 0.3; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));

Tg1 = 0.5; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Elemental | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));

Tg1 = 0.1; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Chunked 2 | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));

Tg1 = 0.3; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Chunked 2 | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));

Tg1 = 0.5; Tgoal(1) = Tg1;
[pi,Kpi,V]  = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
[TX,QX] = OFC_RollOut(xinit,pi,Kpi,A,B,H,R,Q);
OFC_Plot(TX,'Description',sprintf('OT | Chunked 2 | No Perturbation | Target 1 Time = %.2f | Cost = %.2f',Tg1,V));
