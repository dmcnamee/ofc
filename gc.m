function [x,y] = gc(loc,lim,res)
%% FUNCTION: Returns (x,y) grid coordinates for continuous coordinates loc.
% Inputs:   grid    = discretrized grid.
%           res     = resolution of grid.
%           loc     = continuous coordinates to be discretized.
% OUTPUTS:  (x,y)   = discrete coordinates of loc.
% NOTES:    N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

x = ceil((loc(1)-lim(1,1))/res);
y = ceil((loc(2)-lim(2,1))/res);

end
