function OFC_Plot(TX,varargin)
%% FUNCTION: Plots time x state trajectories.
% INPUTS:   TX = time x state trajectory
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   Build plot function that compares two trajectories.
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% variables
global T plim vlim alim ngoal pgoal Tgoal;
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

%% plot settings
set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultAxesFontSize',18);
set(0,'DefaultScatterMarker','+');
%set(0,'DefaultScatterSizeData',50^2);
set(0,'DefaultScatterLineWidth',1);
set(0,'DefaultScatterMarkerFaceColor','red');
set(0,'DefaultScatterMarkerEdgeColor','red');
set(0,'DefaultLineColor','black');
set(0,'DefaultLineLineWidth',3);
set(0,'DefaultLineMarkerSize',20);
set(0,'DefaultSurfaceLineWidth',3);
set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on');

%% plot state
figure();
ind     = reshape(1:6,2,3)';
names   = {'Position','Velocity','Actuation'};
units   = {'cm','cm/s','cm/s$^2$'};
lim.Position = plim; lim.Velocity = vlim; lim.Actuation = alim;
for i=1:3
    subplot(2,2,i); hold on;
    name = names{i}; unit = units{i};
    
    %plot(TX(:,ind(i,1)),TX(:,ind(i,2)),'LineColor',TrajColor,varargin{:});                                     % 2D
    plot_GradientLine(TX(:,ind(i,1)),TX(:,ind(i,2)),T','z',T',varargin{:}); rotate3d on; zlabel('Time (s)');    % 3D, line color = time
    %plot3(TX(:,ind(i,1)),TX(:,ind(i,2)),T',varargin{:}); rotate3d on; zlabel('Time (s)');                      % 3D
    xlabel(sprintf('X %s (%s)',name,unit)); ylabel(sprintf('Y %s (%s)',name,unit));
    title(name);
    if strcmp(name,'Position')
        xlim(lim.(name)(1,:)); ylim(lim.(name)(2,:));   % modulate axis limits
        for n=1:ngoal                                   % plot targets
            %scatter(pgoal(n,1),pgoal(n,2));
            scatter3(pgoal(n,1),pgoal(n,2),Tgoal(n));
        end
        scatter(TX(1,1),TX(1,2),'.');
        scatter3(TX(1,1),TX(1,2),0,'.'); % plot start position
    end  
end
colorbar();
suptitle(desc);

end
