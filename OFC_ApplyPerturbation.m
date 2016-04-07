function [xpert,Pert] = OFC_ApplyPerturbation(ti,x,Pert)
%% FUNCTION: Applies perturbation to tstep/state (ti,x).
% INPUTS:   ti          = input tstep
%           x           = input state
%           Pert.TX     = explicit tstep-state specification of perturbations (tsteps x 3*mdim)
%           Pert.Th.T   = threshold for T constraints
%           Pert.Th.X   = threshold for X constraints
%           Pert.C.T    = time constraint (e.g. time-t'<Th.T)
%           Pert.C.X    = state constraint (e.g. state-x'<Th.X)
%           Pert.P.T    = perturbation corresponding to time constraint (currently not used)
%           Pert.P.X    = perturbation corresponding to state constraint
% OUTPUTS:  xpert       = perturbed state
%           Pert        = perturbation struct with applied perturbation removed
% NOTES:    Each constraint/perturbation will only be applied once.
% ISSUES:   Use more sophisticated logging techniques than "verbose==true".
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
verbose = true;

%% tstep-state perturbations
if isfield(Pert,'TX')
    xpert = x + Pert.TX(:,ti);
else
    xpert = x;
end

if isfield(Pert,'C')
    %% time-constraint based perturbations
    if isfield(Pert.C,'T')
        Pert.C.TSTEPS = arrayfun(time2tstep,Pert.C.T);
        pi = find(Pert.C.TSTEPS == ti);
        if pi
            xpert           = xpert + Pert.P.TX(:,pi);
            Pert.C.TX(:,pi)  = [];                       % remove applied perturbation
            if verbose
                fprintf('\nTime-constraint perturbation applied @TIME = %.2f, @TSTEP = %i:\n',Pert.C.T(pi),ti);
                fprintf('%s\t = [%s]\n','x',sprintf('%.2f ',x));
                fprintf('%s\t = [%s]\n','Pert',sprintf('%.2f ',Pert.P.TX(:,pi)));
                fprintf('%s\t = [%s]\n','xpert',sprintf('%.2f ',xpert));
            end
        end
    end

    %% state-constraint based perturbations
    if isfield(Pert.C,'X')
        npertX = size(Pert.C.X,2);
        for pi=1:npertX
            ind = ~isnan(Pert.C.X(:,pi));                       % ignores nans in comparison
            if norm(Pert.C.X(ind,pi)-x(ind)) < Pert.Th.X(pi);   % threshold parameter
                xpert          = xpert + Pert.P.X(:,pi);
                Pert.C.X(:,pi) = [];                            % remove applied perturbation
                if verbose
                    fprintf('\nState-constraint perturbation applied @TIME = %.2f, @TSTEP = %i:\n',tstep2time(ti),ti);
                    fprintf('%s\t = [%s]\n','x',sprintf('%.2f ',x));
                    fprintf('%s\t = [%s]\n','Pert',sprintf('%.2f ',Pert.P.X(:,pi)));
                    fprintf('%s\t = [%s]\n','xpert',sprintf('%.2f ',xpert));
                end
                break
            end
        end
    end
end

end
