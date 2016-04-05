function V = OFC_LQGSDN_CostToGo(x,Gx,Ge,Sx,Se,s)
%% FUNCTION: OFC LQGSDN cost-to-go.
% INPUTS:   x  = initial state (assumed known precisely)
%           Gx = non-centered covariance of the state estimate ("\Sigma_x")
%           Ge = expected estimation error covariance ("\Sigma_e")
%           Sx = component of cost-to-go due to state
%           Se = component of cost-to-go due to estimation error
%           s  = component of cost-to-go due to independent noise sources
%                (dynamics, sensory feedback, and internal representation)
% OUTPUTS:  V  = expected cost-to-go
% NOTES:    N/A
% ISSUES:   Seems to output quite low numbers - check thoroughly.
% REFS:     Todorov2005 (just after Eqn 4.2)
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

V = x'*Sx*x + trace((Sx+Se)*Ge) + s;

end