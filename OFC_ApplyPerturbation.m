function [xpert,Pert] = OFC_ApplyPerturbation(ti,x,Pert)
%% FUNCTION: Applies perturbation to tstep/state (ti,x).
% INPUTS:   ti              = input tstep
%           x               = input state
%           Pert.TX         = explicit tstep-state specification of perturbations (3*mdim x tsteps)
%           Pert.Th.T       = threshold for T constraints
%           Pert.Th.X       = threshold for X constraints
%           Pert.C.T        = time constraint (e.g. time-t'<Th.T)
%           Pert.C.X        = state constraint (e.g. state-x'<Th.X)
%           Pert.P.T        = perturbation corresponding to time constraint (currently not used)
%           Pert.P.X.pulse  = pulsed perturbation corresponding to state constraint (only applied for one tstep)
%           Pert.P.X.step   = stepped perturbation corresponding to state constraint (does not turn off)
% OUTPUTS:  xpert           = perturbed state
%           Pert            = perturbation struct with applied perturbation removed
% NOTES:    N/A
% ISSUES:   Use more sophisticated logging techniques than "verbose==true".
%           Time-constraint perturbations unfinished.
%           Incorporate step-perturbations - maybe better to encode as part
%           of underlying dynamics.
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global StepPertInit xdim;
verbose = true;

%% tstep-state perturbations
if isfield(Pert,'TX')
    pert = Pert.TX(:,ti);
    if verbose
        fprintf('\nDirectly TX-encoded perturbation applied.\n');
    end
else
    pert = zeros(xdim,1);
end

if isfield(Pert,'C')
    %% time-constraint based perturbations
    if isfield(Pert.C,'T')
        Pert.C.TSTEPS = arrayfun(time2tstep,Pert.C.T);
        pi = find(Pert.C.TSTEPS == ti);
        if pi
            pert = pert + Pert.P.TX(:,pi);
            if verbose
                fprintf('\nTime-constraint perturbation applied @TIME = %.2f, @TSTEP = %i:\n',Pert.C.T(pi),ti);
            end
        end
    end

    %% state-constraint based perturbations
    if isfield(Pert.C,'X')
        npertX = size(Pert.C.X,2);
        for pi=1:npertX
            ind = ~isnan(Pert.C.X(:,pi));                           % ignores nans in comparison
            if norm(Pert.C.X(ind,pi)-x(ind)) < Pert.Th.X(pi);       % threshold parameter
                if isfield(Pert.P.X,'pulse')
                    pert           = pert + Pert.P.X.pulse(:,pi);
                    Pert.C.X(:,pi) = [];                            % remove applied perturbation
                elseif isfield(Pert.P.X,'step')
                    pert           = pert + Pert.P.X.step(:,pi);
                    StepPertInit   = pi;                            % flag step perturbation as initiated (NOT USED FOR NOW)
                end
                if verbose
                    fprintf('\nState-constraint perturbation applied @TIME = %.2f, @TSTEP = %i:\n',tstep2time(ti),ti);
                end
                break
            end
        end
    end
end

%% add to current state
xpert = x + pert;

end
