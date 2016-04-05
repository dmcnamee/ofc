function QS = OFC_CostSpace(S,R,Q)
%% FUNCTION: Outputs cost of all states in state-space S.
% INPUTS:   S  = complete state-space
%           R  = energy cost matrix
%           Q  = goal-targets-costs matrix
% OUTPUTS:  QS = cost space
% NOTES:    S,QS typically exceed array size limits.
% ISSUES:   Better vectorization?
% REFS:     Todorov2002*
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global psteps vsteps usteps;
QS = nan(psteps,psteps,vsteps,vsteps,usteps,usteps);

%% for-loop to compute cost-space
fprintf('OFC_ComputeCostSpace: computing via for-loop...\n');
tic;
for pi1=1:psteps
    for pi2=1:psteps
        for vi1=1:vsteps
            for vi2=1:vsteps
                for ui1=1:usteps
                    for ui2=1:usteps
                        x = S(pi1,pi2,vi1,vi2,ui1,ui2,:);
                        QS(pi1,pi2,vi1,vi2,ui1,ui2) = OFC_ComputeCost(x,R,Q);
                    end
                end
            end
        end
    end
end
fprintf('OFC_ComputeCostSpace: '); toc;

end
