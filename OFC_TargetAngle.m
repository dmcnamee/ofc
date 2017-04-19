function [ secondtarget ] = OFC_TargetAngle(pvia, pangle, pdist2 )
%This function generates the viapoint and secondary target positions as
%goals for a reaching motor task.
% INPUTS:   pvia      = cartesian position of via target
%           pangle    = angle (in degrees) of second target, measured CW from North
%           pdist2    = distance between via target and second target
% OUTPUTS:  secondtarget = cartesian position of second target location
% NOTES:    N/A
% ISSUES:   N/A
% AUTHOR:   Hannah Sheahan, sheahan.hannah@gmail.com

secondtarget = pvia + sign(pangle).*pdist2.*[-sind(pangle-90), cosd(pangle-90)]; % x, y position

end

