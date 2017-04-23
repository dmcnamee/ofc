function P = TrajectoryProbability(TX,L,K,A,B,H,varargin)
%% FUNCTION: Probability of trajectory TX under control law L and sensory filter K.
% INPUTS:   TX          = time x state input matrix
%           L           = control law
%           K           = Sensory filter
%           A           = state-evolution matrix
%           B           = control-input matrix
%           H           = sensory-indicator matrix
% OUTPUTS:  P           = state probability as a function of time
% NOTES:    Viterbi path... Need to specify noise levels.
% ISSUES:   To do.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% roll out expected trajectory
ETX = TrajectoryRollOut(x,L,K,A,B,H,varargin{:});



end