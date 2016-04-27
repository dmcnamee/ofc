function S = OFC_StateSpace()
%% FUNCTION: Computes discretized state-space.
% INPUTS:   N/A
% OUTPUTS:  S = discretized state space, S(p1,p2,v1,v2,u1,u2,:) = state vector
% NOTES:    S typically exceeds array size limits. 
% ISSUES:   N/A
% REFS:     http://uk.mathworks.com/matlabcentral/newsreader/view_thread/269186
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global pres vres ures plim vlim ulim psteps vsteps usteps;

P1 = plim(1,1):pres:plim(1,2);
P2 = plim(2,1):pres:plim(2,2);
V1 = vlim(1,1):vres:vlim(1,2);
V2 = vlim(2,1):vres:vlim(2,2);
U1 = ulim(1,1):ures:ulim(1,2);
U2 = ulim(2,1):ures:ulim(2,2);

%% construct array
[P1i,P2i,V1i,V2i,U1i,U2i] = ndgrid((1:psteps)',(1:psteps)',(1:vsteps)',(1:vsteps)',(1:usteps)',(1:usteps)');
S = [P1(P1i(:),:),P2(P2i(:),:),V1(V1i(:),:),V2(V2i(:),:),U1(U1i(:),:),U2(U2i(:),:)];

%% reshape

end
