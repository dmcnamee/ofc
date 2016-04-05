function Qt = OFC_TaskConstraintCostMatrix(t,Q)
%% FUNCTION: Retrieves task-constraint cost matrix at time-step t.
% INPUTS:   t  = time-step
%           Q  = goal-targets-costs matrix
% OUTPUTS:  Qt = constraint cost matrix (e.g. goal-targets) for time-step t
% NOTES:    N/A
% REFS:     Todorov2002*
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global Tgoal wgoal xdim;
ti      = time2tstep(t);
Tgoali  = arrayfun(@time2tstep,Tgoal);

goali = find((ti==Tgoali).*(wgoal~=0));     % test whether via-point-goal within same time-step AND has a non-zero weight (necessary for LQGSDN_Chunked)
if goali
    Qt = squeeze(Q(goali,:,:));             % via-point reward
else
    Qt = zeros(xdim,xdim);
end

end
