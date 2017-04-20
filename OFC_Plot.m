function OFC_Plot(TX,varargin)
%% FUNCTION: Plots time x state trajectories.
% INPUTS:   TX = time x state trajectory
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Build plot function that compares two trajectories.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mccompe@gmail.com
      
%% variables
while ~isempty(varargin)
    switch varargin{1}
        case 'Description'
            desc = varargin{2};
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
for i=1:3
    subplot(2,2,i); hold on;
    OFC_SubPlot(TX,components{i});
end
subplot(2,2,4); colorbar();
suptitle(desc);

end
