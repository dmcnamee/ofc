function J = ofc_MDP_costfunc(u,p,v,t)
%% FUNCTION: MDP cost-function.
% Inputs:   u   = control input.
%           p   = position.
%           v   = velocity.
%           t   = time point.
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Incomplete.
% REFS:     Todorov2002 / Liu2007* / Nashed2012 / Nashed2014
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
% ISSUES:   How to prevent optimal controller sticking at target? vmin?
%           Use modular dynamic programming architecture?

%% global variables
global pgoal goalsize;
global Wenergy Wtime Wgoal Wtimeout;
global tmax vmax;

J = 0;

%% energy cost
J = J + Wenergy*(norm(u)^2);    % energy cost

%% task-constraint costs
if (t<tmax) && (norm(v)<vmax)
    J = J + Wtime*t;            % time cost
    for n=1:ngoals,             % goal cost
        if norm(pgoal(n,:)-p) < goalsize;
            J = J + Wgoal;
        end
    end
else
    J = J + Wtimeout;           % timeout cost
end

end
