function [R,Q] = OFC_LQG_costfunc()
%% FUNCTION: LQG via-point cost-function.
% INPUTS:   N/A
% OUTPUTS:  R = energy cost matrix
%           Q = goal-targets-costs matrix
% NOTES:    ngoal is not changed in order to keep the state dimensionality
%           the same throughout whether deriving chunked/elemental
%           solutions.
% ISSUES:   N/A
% REFS:     Todorov2002* / Liu2007 / Nashed2012 / Nashed2014 / Osu2015
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global xdim Wenergy Wgoal mdim wgoal ngoal Wstop Wactuator;
ngoalS = sum(wgoal~=0);             % number of goals currently "active"

%% energy cost matrix
R = Wenergy*eye(mdim,mdim);

%% via-point costs matrix
Q = zeros(ngoal,xdim,xdim);
for g=1:ngoal-1
    D        = zeros(2,xdim);       % via-point cost matrix
    D(1,1)   = -1*Wgoal*wgoal(g); D(1,3*mdim+2*(g-1)+1) = 1*Wgoal*wgoal(g);
    D(2,2)   = -1*Wgoal*wgoal(g); D(2,3*mdim+2*(g-1)+2) = 1*Wgoal*wgoal(g);
    Q(g,:,:) = 1/4*(D'*D);
end
D      = zeros(mdim*3,xdim);        % final stop-point cost matrix
D(1,1) = -1*Wgoal*wgoal(ngoal); D(1,3*mdim+2*(ngoal-1)+1) = 1*Wgoal*wgoal(ngoal);
D(2,2) = -1*Wgoal*wgoal(ngoal); D(2,3*mdim+2*(ngoal-1)+2) = 1*Wgoal*wgoal(ngoal);
D(3,3) = Wstop; D(4,4) = Wstop;
D(5,5) = Wactuator; D(6,6) = Wactuator;
Q(ngoal,:,:) = 1/(ngoalS+2)*(D'*D);

end
