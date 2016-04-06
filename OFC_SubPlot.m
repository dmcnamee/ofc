function OFC_SubPlot(TX,component,varargin)
%% FUNCTION: Plots component of time x state trajectories.
% INPUTS:   TX        = time x state trajectory
%           component = string indicating state component to plot
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global T plim vlim alim ngoal pgoal Tgoal;
ind          = reshape(1:6,2,3)';
components   = {'Position','Velocity','Actuation'};
units        = {'cm','cm/s','cm/s$^2$'};
lim.Position = plim; lim.Velocity = vlim; lim.Actuation = alim;

%% plot settings
OFC_PlotSettings();

%% set component to be plotted
i    = find(ismember(components,component));
comp = components{i};
unit = units{i};

%% plot state component
%     plot(TX(ind(i,1),:),TX(ind(i,2),:),'LineColor',TrajColor,varargin{:});                                    % 2D
plot_GradientLine(TX(ind(i,1),:),TX(ind(i,2),:),T,'z',T,varargin{:}); rotate3d on; zlabel('Time (s)');    % 3D, line color = time
%     plot3(TX(ind(i,1),:),TX(ind(i,2),:),T',varargin{:}); rotate3d on; zlabel('Time (s)');                     % 3D
if strcmp(comp,'Position')
    xlim(lim.(comp)(1,:)); ylim(lim.(comp)(2,:));   % modulate axis limits
    for n=1:ngoal                                   % plot targets
        scatter3(pgoal(n,1),pgoal(n,2),Tgoal(n));
    end
    scatter(TX(1,1),TX(2,1),'.');
    scatter3(TX(1,1),TX(2,1),0,'.');                % plot start position
end
xlabel(sprintf('X %s (%s)',comp,unit)); ylabel(sprintf('Y %s (%s)',comp,unit));
title(comp);

end