function q = OFC_ComputeCost(x,R,Q)
%% FUNCTION: Computes cost of state x.
% INPUTS:   x = state
%           R = energy cost matrix
%           Q = goal-targets-costs matrix
% OUTPUTS:  q = output cost
% NOTES:    Currently, only for quadratic costs.
%           Decompose Q as a function task constraints?
% REFS:     Todorov2002*
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

costForm = 'quadratic';

if strcmp(costForm,'quadratic')
    u = x(5:6); % extract control/actuation sub-state
    q = u'*R*u + x'*Q*x;
else
    error('OFC_ComputeCost: unknown form of cost function.\n');
end

if isnan(q)
    error('OFC_ComputeCost: nan returned.');
end

end
