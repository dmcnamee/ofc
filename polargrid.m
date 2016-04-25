function [X,Y] = polargrid(radii,angles)
%% FUNCTION: Returns polar grid in cartesian coordinates.
% INPUTS:   radii   = radii defining grid positions.
%           angles  = angles (in radians) define grid positions.
% OUTPUTS:  X,Y     = x,y positions
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

[r, phi] = meshgrid(radii, angles);
X = r.*cos(phi);
Y = r.*sin(phi);

end