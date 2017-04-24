function OFC_Plot(TX,varargin)
%% FUNCTION: Plots time x state trajectories.
% INPUTS:   TX = time x state trajectory
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Build plot function that compares two trajectories.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mccompe@gmail.com
      
%% variables
global mdim xdim tsteps;
while ~isempty(varargin)
    switch varargin{1}
        case 'Description'
            desc = varargin{2};
        case 'L'
            L = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}]);
    end
      varargin(1:2) = [];
end
if ~exist('desc','var')
    desc = 'Optimal Trajectory';
end
components = {'Position','Velocity','Actuation'};

%% plot settings
OFC_PlotSettings();

%% plot state
figure();
suptitle(desc);
for i=1:3
    subplot(2,2,i); hold on;
    OFC_SubPlot(TX,components{i});
end
if exist('L','var')
    subplot(2,2,4);
    L = L(1:mdim,1:mdim,:); % reduce to position-related gains
    Lflat = reshape(L,[mdim mdim*tsteps]);
%     Lflat = reshape(L,[mdim xdim*tsteps]); 
    V = cov(Lflat);
    imshow(V);
    colorbar();
    xlabel('Dim 1');
    ylabel('Dim 2');
    title('Gain Covariance');
end


end
