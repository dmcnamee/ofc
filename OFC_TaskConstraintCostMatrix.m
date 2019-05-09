function Qt = OFC_TaskConstraintCostMatrix(t,Q)
%% FUNCTION: Retrieves task-constraint cost matrix at time-step t.
% INPUTS:   t  = time-step
%           Q  = goal-targets-costs matrix
% OUTPUTS:  Qt = constraint cost matrix (e.g. goal-targets) for time-step t
% NOTES:    N/A
% REFS:     Todorov2002*
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
% EDITED:   Hannah Sheahan, sheahan.hannah@gmail.com (Oct-2017)

%% variables
global Tgoal Tatgoal tres wgoal xdim;
ti      = time2tstep(t);
Tgoali  = arrayfun(@time2tstep,Tgoal);                      % timestep index for the goal to be hit
tstepsatgoal = ceil(Tatgoal./tres);
atgoal = ((Tgoali-ti)>=0) & ((Tgoali-ti)<=tstepsatgoal);    % should be at goal for this required time
goali = find(atgoal.*(wgoal~=0));

if goali  
    Qt = squeeze(Q(goali,:,:));                             % via-point reward
else
    Qt = zeros(xdim,xdim);
end

end
