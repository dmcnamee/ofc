function OFC_SetPGOAL(pgoalNew)
%% FUNCTION: Sets pgoal and adapts necessary parameters/variables.
% INPUTS:   pgoal = new goal-target positions
% OUTPUTS:  N/A
% NOTES:    Does not currently allow for changes in number of goal-targets.
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

global xinit pgoal ngoal mdim;
pgoal = pgoalNew;
OFC_GlobalVars(); % set variable dimensions based on ngoal
xinit = [xinit(1:6)' reshape(pgoal',1,ngoal*mdim)]';

end