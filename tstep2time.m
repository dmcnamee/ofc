function t = tstep2time(ti)
%% FUNCTION: Converts a tstep ti to a time t.
% INPUTS:   ti  = tstep
% OUTPUTS:  t   = time
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global T;

%% convert
t = T(ti);

end
