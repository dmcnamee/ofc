function [Ppmf,Upmf] = ofc_MDP_dynamics(U)
%% FUNCTION: OFC dynamics of MDP model (state-space formalism).
% Inputs:   b    = viscous constant, default = 10 N.s/m
%           tau  = time constant of linear filter, default = 50ms
%           m    = mass of hand, default = 1kg
%           U    = control input space
%           P    = position space
%           V    = velocity space
% OUTPUTS:  Ppmf = distribution over position/velocity space for next timestep
%           Upmf = distribution over control signal (due to signal-dependent noise)
% NOTES:    N/A
% ISSUES:   Incomplete.
% REFS:     Todorov2002 / Liu2007* / Nashed2012 / Nashed2014
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% setup variables
global mdim;
global plim vlim ulim;
global tres pres vres ures;
global pgoal goalsize ngoal;
global b tau m tf;
global tsteps psteps vsteps usteps;
global c1 c2 C usigma;
global Wenergy Wtime Wstop Wgoal Wtimeout;
global tmax vmax;
P   = [plim(1,1):pres:plim(1,2) ; plim(2,1):pres:plim(2,2)];    % discrete position space
V   = [vlim(1,1):vres:vlim(1,2) ; vlim(2,1):vres:vlim(2,2)];    % discrete velocity space
Vpmf= nan(vsteps(1),vsteps(2),usteps(1),usteps(2),vsteps(1),vsteps(2));
Ppmf= nan(psteps(1),psteps(2),vsteps(1),vsteps(2),size(U,1),size(U,2),psteps(1),psteps(2),vsteps(1),vsteps(2)); % current P/V/U, next P/V         
Upmf= nan(size(U,1),size(U,2));                                 % prob mass function over control signal

%% compute signal dependent noise
% sampling
% w = normrnd(0,usigma,mdim,1);    % brownian noise
% M = bsxfun(@times,C,u)*w;        % control-dependent noise correlations

%% compute pmf for signal-dependent noise
% tau.du = u.dt + M.dw
% unext = u + M;
for i=1:mdim
    Upmf(i,:) = normpdf(U(i,:),U(i,:),usigma*(c1*U(i,:)+c2*U(i,:)));
end
Upmf = pnorm(Upmf,1);

%% update velocity state
% m*dv = (u - b.v).dt
% vnext = tres*(u - b*v)/m;
V(1,:) = tres(U-b*
Spmf(:,:,X,

for vxi=1:vsteps(1)
    for vyi=1:vsteps(2)
        for uxi=1:usteps(1)
            for uyi=1:usteps(2)
                VXpmf = find();
                VYpmf = 
                Vpmf(vxi,vyi,uxi,uyi,:,:) = kron(VXpmf,VYpmf');
            end
        end
    end
end

Vpmf = (U+b*V)*m/tres

%% update position state
% dp = v.dt
% pnext = tres*v + p;



end
