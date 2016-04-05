%% FUNCTION: LTI MDP model with control-signal-dependent noise.
% Inputs:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Incomplete.
% REFS:     Todorov2002 / Liu2007* / Nashed2012 / Nashed2014
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com
% TOOLBOXES:    Control Systems Toolbox (http://uk.mathworks.com/products/control/)
%               MDPtoolbox (http://www7.inra.fr/mia/T/MDPtoolbox/)

%% set global parameters
clear all; close all; clc;
global mdim;
global plim vlim ulim;
global tres pres vres ures;
global pgoal goalsize ngoal;
global b tau m tf;
global tsteps psteps vsteps usteps;
global c1 c2 C usigma;
global Wenergy Wtime Wstop Wgoal Wtimeout;
global tmax vmax;
varargin = {};
ofc_parameters(varargin{:});


%% setup variables
p       = nan(mdim,tsteps);   % grid position of hand
v       = nan(mdim,tsteps);   % grid velocity of hand
u       = nan(mdim,tsteps);   % control signal
pi      = nan(mdim,tsteps);   % optimal policy

%% discretize state-space
T           = 0:tmax:tres;                                              % time space
P           = [plim(1,1):pres:plim(1,2) ; plim(2,1):pres:plim(2,2)];    % discrete position space
V           = [vlim(1,1):vres:vlim(1,2) ; vlim(2,1):vres:vlim(2,2)];    % discrete velocity space
U           = [ulim(1,1):ures:ulim(1,2) ; ulim(2,1):ures:ulim(2,2)];    % discrete control signal space
PGOAL       = P*0;                                                      % discrete goal-target space
for n=1:ngoal
    [x,y]       = gc(pgoal(n,:),plim,pres);
    PGOAL(1,x)  = 1; PGOAL(2,y) = 1;
end

% % discrete grids
% T           = 0:tmax:tres;                                               % time space
% [PX,PY]     = ndgrid(plim(1,1):pres:plim(1,2),plim(2,1):pres:plim(2,2)); % position grid
% [VX,VY]     = ndgrid(vlim(1,1):vres:vlim(1,2),vlim(2,1):vres:vlim(2,2)); % velocity grid
% [UX,UY]     = ndgrid(ulim(1,1):ures:ulim(1,2),ulim(2,1):ures:ulim(2,2)); % control signal grid
% PGOAL       = PgridX*0;                                                  % goal-target position grid
% for n=1:ngoal
%     [x,y]       = gc(pgoal(n,:),plim,pres);
%     PGOAL(x,y)  = 1;
% end

%% dynamic programming
J = nan(usteps(1),usteps(2),psteps(1),psteps(2),vsteps(1),vsteps(2),tsteps); 

for ti=fliplr(1:tsteps)
    for uxi=fliplr(1:usteps(1))
        for uyi=fliplr(1:usteps(2))
            for pxi=fliplr(1:psteps(1))
                for pyi=fliplr(1:psteps(2))
                    for vxi=fliplr(1:vsteps(1))
                        for vyi=fliplr(1:vsteps(2))
                            % set state-space variables
                            ux = U(1,uxi); uy = U(2,uyi);
                            px = P(1,pxi); py = P(2,pyi);
                            vx = V(1,vxi); vy = V(2,vyi);
                            t  = T(ti);

                            if t==T(end),
                                J(u,p,v,t) = ofc_MDP_costfunc(u,p,v,t); % final timepoint
                            else
                                % dynamics
                                D = ;
                                
                                % update
                                J(u,p,v,t) = ofc_MDP_costfunc(u,p,v,t) + sum(D*J(u,p,v,t+1));
                            end
                        end
                    end
                end
            end
        end
    end
end
