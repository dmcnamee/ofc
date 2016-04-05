%% FUNCTION: OFC-LQGSDN testing multiple temporal adjustments.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Need to impose time cost.
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
clear all; close all; clc;
OFC_Parameters();
pgoal1      = [0 10];
pgoal2      = [0 10; -5 10];
% pgoal3      = [0 10; -5 20; -5 10];
% pgoal3      = [0 10; -5 10; -5 0];
pgoal3      = [0 10; -5 0; -10 0];
pgoal       = pgoal2;
OFC_Parameters('pgoal',pgoal);
Chunks2     = [1 1];
Chunks3     = [1 1 0; 0 0 1];
Chunks      = Chunks2;
Tgoal1Adj   = 0.4;
Tgoal1Range = 0.05:0.02:0.5; % range of Tgoal(1) to test
Tgoal2Range = 0.55:0.02:1.15; % range of Tgoal(2) to test
Tgoal3Range = 1.0:0.02:1.0;  % range of Tgoal(3) to test
global xdim Tgoal xinit;
Tdef        = Tgoal;     % default Tgoal time
tiStitch    = 6;            % perturbation tstep
x           = xinit;

% define state-based perturbation
Pert.C.X      = nan(1,xdim);
Pert.Th.X(1)  = 1.0;
Pert.C.X(1,2) = 1;               % if norm(y,5)<pres
Pert.P.X      = zeros(1,xdim);
%Pert.P.X(1)   = -1;             % perturb px
Pert.P.X(3)   = -10;             % perturb vx

% dynamics
[A,B,C] = OFC_LQGSDN_dynamics();
[H,O]   = OFC_LQG_feedback();
[R,Q]   = OFC_LQG_costfunc();


%% Three Targets, Unperturbed Trajectories
% figure();
% for ti1=1:numel(Tgoal1Range)
%     for ti2=1:numel(Tgoal2Range)
%         Tg1 = Tgoal1Range(ti1);
%         Tg2 = Tgoal2Range(ti2);
%         
%         % elemental
%         OFC_Parameters('pgoal',pgoal);
%         global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
%         [piE,KpiE] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
%         global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
%         [TXE,QXE] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);
% 
%         % 2+1 chunks
%         global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
%         [piC,KpiC]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);
%         global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
%         [TXC,QXC]   = OFC_RollOut(x,piC,KpiC,A,B,H,R,Q);
% 
%         % record results
%         QTg(ti1,ti2,1) = nansum(QXE);
%         QTg(ti1,ti2,2) = nansum(QXC);
%     end
% end
% subplot(2,2,1); hold on;
% plot(QTg(:,1),QTg(:,2),'.-');
% xlabel('Cumulative cost for elemental OFC');
% ylabel('Cumulative cost for chunked OFC');
% title('Unperturbed Trajectory');
% subplot(2,2,2); hold on;
% plot(Tgoal1Range,QTg(:,1),'.-');
% plot(Tgoal1Range,QTg(:,2),'.-');
% xlabel('Time to target 1 (s)');
% ylabel('Cumulative cost');
% legend('Elemental OFC','2+1 Chunked OFC');
% title('Unperturbed Trajectory');

%% perturbed version goes here

% subplot(2,2,3); hold on;
% plot(QTg(:,1),QTg(:,2),'.-');
% xlabel('Cumulative cost for elemental OFC');
% ylabel('Cumulative cost for chunked OFC');
% title('Perturbed Trajectory');
% subplot(2,2,4); hold on;
% plot(Tgoal1Range,QTg(:,1),'.-');
% plot(Tgoal1Range,QTg(:,2),'.-');
% xlabel('Time to target 1 (s)');
% ylabel('Cumulative cost');
% title('Perturbed Trajectory');
% legend('Elemental OFC','2+1 Chunked OFC');



%% Varying multiple target times
for ti1=1:numel(Tgoal1Range)
    for ti2=1:numel(Tgoal2Range)
        Tg1 = Tgoal1Range(ti1);
        Tg2 = Tgoal2Range(ti2);
        
        % 2+1 chunks without perturbation
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [piC,KpiC]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [TXC,QXC]   = OFC_RollOut(x,piC,KpiC,A,B,H,R,Q);

        % three points, elemental without perturbation
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [piE,KpiE] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [TXE,QXE] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q);

        % record results
        QTg(ti1,ti2,1) = nansum(QXE);
        QTg(ti1,ti2,2) = nansum(QXC);
    end
end
[T1E,T2E] = ind2sub(size(QTg(:,:,1)),argmin(QTg(:,:,1)));
[T1C,T2C] = ind2sub(size(QTg(:,:,2)),argmin(QTg(:,:,2)));
t1EOpt = Tgoal1Range(T1E); t2EOpt = Tgoal2Range(T2E);
t1COpt = Tgoal1Range(T1C); t2COpt = Tgoal2Range(T2C);

for ti1=1:numel(Tgoal1Range)
    for ti2=1:numel(Tgoal2Range)
        Tg1 = Tgoal1Range(ti1);
        Tg2 = Tgoal2Range(ti2);
        %2+1 chunks with perturbation
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = t1COpt; Tgoal(2) = t2COpt;
        [piCOpt,KpiCOpt]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);                     % optimal timing
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [piCAdj,KpiCAdj]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);                     % adjusted timing
        % identify perturbation timepoint for policy stitching
        TX = OFC_RollOut(x,piCOpt,KpiCOpt,A,B,H,R,Q);
        tiStitch = argmin(abs(TX(1:20,2) - Pert.C.X(1,2)));
        [piC,KpiC] = OFC_StitchPolicies(tiStitch,piCOpt,KpiCOpt,piCAdj,KpiCAdj);
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [TXC,QXC]   = OFC_RollOut(x,piC,KpiC,A,B,H,R,Q,'Perturbations',Pert);

        % three points, elemental with perturbation
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = t1EOpt; Tgoal(2) = t2EOpt;  % optimal timing
        [piEOpt,KpiEOpt] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;        % adjusted timing
        [piEAdj,KpiEAdj] = OFC_LQGSDN_Elemental(x,A,B,C,H,O,R,Q);
        % identify perturbation timepoint for policy stitching
        TX = OFC_RollOut(x,piEOpt,KpiEOpt,A,B,H,R,Q);
        tiStitch = argmin(abs(TX(1:20,2) - Pert.C.X(1,2)));
        [piE,KpiE] = OFC_StitchPolicies(tiStitch,piEOpt,KpiEOpt,piEAdj,KpiEAdj);
        OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1; Tgoal(2) = Tg2;
        [TXE,QXE] = OFC_RollOut(x,piE,KpiE,A,B,H,R,Q,'Perturbations',Pert);

        % record results
        QTgP(ti1,ti2,1) = nansum(QXE);
        QTgP(ti1,ti2,2) = nansum(QXC);
    end
end
[T1E,T2E] = ind2sub(size(QTg(:,:,1)),argmin(QTg(:,:,1)));
[T1C,T2C] = ind2sub(size(QTg(:,:,2)),argmin(QTg(:,:,2)));
[T1EP,T2EP] = ind2sub(size(QTgP(:,:,1)),argmin(QTgP(:,:,1)));
[T1CP,T2CP] = ind2sub(size(QTgP(:,:,2)),argmin(QTgP(:,:,2)));
[XTg,YTg] = meshgrid(Tgoal1Range,Tgoal2Range);
subplot(2,2,1); hold on;
surface(XTg,YTg,QTg(:,:,1)');
scatter3(Tgoal1Range(T1E),Tgoal2Range(T2E),argmin(QTg(:,:,1)));
title('Elemental | No Perturbation');
zlabel('Cumulative Cost');
xlim([Tgoal1Range(1) Tgoal1Range(end)]);
ylim([Tgoal2Range(1) Tgoal2Range(end)]);
subplot(2,2,2); hold on;
surface(XTg,YTg,QTg(:,:,2)');
scatter3(Tgoal1Range(T1C),Tgoal2Range(T2C),argmin(QTg(:,:,2)));
title('Chunked | No Perturbation');
zlabel('Cumulative Cost');
xlim([Tgoal1Range(1) Tgoal1Range(end)]);
ylim([Tgoal2Range(1) Tgoal2Range(end)]);
subplot(2,2,3); hold on;
surface(XTg,YTg,QTgP(:,:,1)');
scatter3(Tgoal1Range(T1EP),Tgoal2Range(T2EP),argmin(QTgP(:,:,1)));
title('Elemental | Perturbation');
zlabel('Cumulative Cost');
xlim([Tgoal1Range(1) Tgoal1Range(end)]);
ylim([Tgoal2Range(1) Tgoal2Range(end)]);
subplot(2,2,4); hold on;
surface(XTg,YTg,QTgP(:,:,2)');
scatter3(Tgoal1Range(T1EP),Tgoal2Range(T2EP),argmin(QTgP(:,:,2)));
title('Chunked | Perturbation');
zlabel('Cumulative Cost');
xlim([Tgoal1Range(1) Tgoal1Range(end)]);
ylim([Tgoal2Range(1) Tgoal2Range(end)]);

suplabel('Target 1 Time (s)','x');
suplabel('Target 2 Time (s)','y');
suptitle('OT | Perturbation | Temporal Adjustment');




% % test
% Tg1 = 0.35;
% OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tgoal(1);
% [piCDef,KpiCDef,VDef]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);         % default timing
% fprintf('OFC_LQFSDN: expected cost-to-go = %.3f.\n',VDef);
% OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1;
% [piCAdj,KpiCAdj,VAdj]  = OFC_LQGSDN_Chunked(Chunks,x,A,B,C,H,O,R,Q);         % adjusted timing
% fprintf('OFC_LQFSDN: expected cost-to-go = %.3f.\n',VAdj);
% [piC,KpiC] = OFC_StitchPolicies(tiStitch,piCDef,KpiCDef,piCAdj,KpiCAdj);
% OFC_Parameters('pgoal',pgoal); global Tgoal; Tgoal(1) = Tg1;
% [TXC,QXC]   = OFC_RollOut(x,piC,KpiC,A,B,H,R,Q,'Perturbations',Pert);