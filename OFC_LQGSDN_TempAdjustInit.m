%% FUNCTION: OFC-LQGSDN testing temporal adjustments for different initial states.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Chunked not working.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
% clear all; close all; clc;
OFC_Parameters();
pgoal1      = [0 10]';
pgoal2      = [0 10; -5 10]';
% pgoal3      = [0 10; -5 20; -5 10]';
% pgoal3      = [0 10; -5 10; -5 0]';
pgoal3      = [0 10; -5 0; -10 0]';
pgoal       = pgoal2;
OFC_Parameters('pgoal',pgoal); global xdim;
Chunks2     = [1 1];
Chunks3     = [1 1 0; 0 0 1];
Chunks      = Chunks2;
Tgoal1Range = 0.05:0.02:0.4; % range of Tgoal(1) to test
Tgoal2Range = 0.55:0.02:1.15; % range of Tgoal(2) to test
Tgoal3Range = 1.0:0.02:1.0;  % range of Tgoal(3) to test

% define different initial states
xinit1 = zeros(xdim,1);
xinit2 = zeros(xdim,1);
xinit2(4) = -10;

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();

% default initial state
xinit = xinit1;
for ti=1:numel(Tgoal1Range)
    Tg1 = Tgoal1Range(ti);
    
    % elemental
    OFC_Parameters('pgoal',pgoal);
    global Tgoal; Tgoal(1) = Tg1;
    [piE,KpiE,VE]  = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
    
    % 2 chunks
    OFC_Parameters('pgoal',pgoal);
    global Tgoal; Tgoal(1) = Tg1;
    [piC,KpiC,VC]  = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    
    % record results
    QT(1,1,ti) = VE;
    QT(1,2,ti) = VC;
end

% % perturbed initial state
% xinit = xinit2;
% for ti=1:numel(Tgoal1Range)
%     Tg1 = Tgoal1Range(ti);
%     
%     % elemental
%     OFC_Parameters('pgoal',pgoal);
%     global Tgoal; Tgoal(1) = Tg1;
%     [piE,KpiE,VE]  = OFC_LQGSDN_Elemental(xinit,A,B,C,H,O,R,Q);
%     
%     % 2 chunks
%     OFC_Parameters('pgoal',pgoal);
%     global Tgoal; Tgoal(1) = Tg1;
%     [piC,KpiC,VC]  = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
%     
%     % record results
%     QT(2,1,ti) = VE;
%     QT(2,2,ti) = VC;
% end

% QT = tstep x xinit x E/C
T1E = argmin(QT(1,1,:));
T1C = argmin(QT(1,2,:));
% T2E = argmin(QT(2,1,:));
% T2C = argmin(QT(2,2,:));

figure(); hold on;
plot(Tgoal1Range,squeeze(QT(1,1,:)),'.-');
% plot(Tgoal1Range,squeeze(QT(1,2,:)),'.-');
% plot(Tgoal1Range,squeeze(QT(2,1,:)),'.-');
% plot(Tgoal1Range,squeeze(QT(2,2,:)),'.-');
xlabel('Target 1 time (s)'); ylabel('Expected cumulative cost');
legend('xinit1, E', 'xinit1, C', 'xinit2 E', 'xinit2 C');
title('OT | Perturbation | Temporal Adjustment');