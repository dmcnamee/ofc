function [L,K] = OFC_StitchPolicies(ti,L1,K1,L2,K2)
%% FUNCTION: "Stitches" two control policies together.
% INPUTS:   ti          = tstep index of stitch
%           [L1,K1] = first control policy and Kalman filter
%           [L2,K2] = second control policy and Kalman filter
% OUTPUTS:  [L,K]   = combined control policy and Kalman filter
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     Todorov2002* / Liu2007
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% checks
if (isequal(size(L1),size(L2))&&isequal(size(K1),size(K2)))
    % stitch for same number of tsteps
    L = L1; L(:,:,ti+1:end) = L2(:,:,ti+1:end);
    K = K1; K(:,:,ti+1:end) = K2(:,:,ti+1:end);
else
    % stitch for different number of tsteps
    L = L1; L(:,:,ti+1:size(L2,3)) = L2(:,:,ti+1:end);
    K = K1; K(:,:,ti+1:size(K2,3)) = K2(:,:,ti+1:end);
    disp('warning: L/K arrays of unequal size, maybe different tsteps.');
end

end