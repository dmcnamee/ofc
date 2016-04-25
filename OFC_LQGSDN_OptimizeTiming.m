function [piOpt,KpiOpt,TOpt] = OFC_LQGSDN_OptimizeTiming(TMod,TgoalRange,A,B,C,H,O,R,Q,varargin)
%% FUNCTION: Optimize over Tgoal transition times.
% INPUTS:   TMod        = Viapoint goal timing to modulate.
%           TgoalRange  = Range of timings to test.
% OUTPUTS:  piOpt       = optimal control law.
%           KpiOpt      = Optimal Kalman filter.
%           TOpt        = Optimal transition time(s).
% NOTES:    N/A
% ISSUES:   Adapt to multiple TMods.
%           Can only handle one perturbation.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global xdim xinit ngoal Tgoal;

while ~isempty(varargin)
    switch varargin{1}
        case 'Chunks'
            Chunks  = varargin{2};
        case 'Perturbations'
            Pert    = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
      varargin(1:2) = [];
end

%% defaults
if ~exist('Chunks','var')
    Chunks = eye(ngoal,ngoal);      % elemental response
end
if ~exist('Pert','var')
    Pert.P.X = zeros(xdim,1);       % no perturbation
end

%% unperturbed case
Vopt = inf;
for ti=1:numel(TgoalRange)
    Tgoal(TMod) = TgoalRange(ti);
    [pi,Kpi,V] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);
    if (V <= Vopt)
        piOpt   = pi;
        KpiOpt  = Kpi;
        TOpt    = Tgoal(TMod);
        Vopt    = V;
    end
end

%% perturbed case
sumQOpt = inf;
if all(Pert.P.X(:) ~= 0)
    % identify perturbation timepoint for policy stitching
    TX          = OFC_RollOut(xinit,piOpt,KpiOpt,A,B,H,R,Q);                                % roll out
    tiStitch    = find(sum(abs(TX-ones(rows(TX),1)*Pert.C.X),2)<Pert.Th.X);                 % estimate expected time of perturbation

    for ti=1:numel(TgoalRange)
        Tgoal(TMod) = TgoalRange(ti);
        [pi,Kpi] = OFC_LQGSDN_Chunked(Chunks,xinit,A,B,C,H,O,R,Q);                          % compute policy for stitching after perturbation
        [piAdj,KpiAdj]  = OFC_StitchPolicies(tiStitch,piOpt,KpiOpt,pi,Kpi);                 % stitch adjusted policy
        [TXAdj,QXAdj]   = OFC_RollOut(xinit,piAdj,KpiAdj,A,B,H,R,Q,'Perturbations',Pert);   % rollout and measure experienced cost of stitched policy
        sumQ            = sum(QXAdj);

        if (sumQ <= sumQOpt)
            piOpt   = piAdj;
            KpiOpt  = KpiAdj;
            TOpt    = Tgoal(TMod);
            sumQ    = sumQOpt;
        end
    end
end

end