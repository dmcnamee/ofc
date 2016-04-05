function ti = time2tstep(t)
%% FUNCTION: Converts a time t to a timestep ti.
% INPUTS:   t   = time
% OUTPUTS:  ti  = tstep
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global T;

%% convert
ti = argmin(abs(t-T));

end
