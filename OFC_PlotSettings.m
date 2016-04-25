function OFC_PlotSettings()
%% FUNCTION: Set settings for plots.
% INPUTS:   N/A
% OUTPUTS:  N/A
% NOTES:    N/A
% ISSUES:   N/A
% REFS:     N/A
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultAxesFontSize',14);
%set(0,'DefaultAxesLabelFontSize',13);  % causes labels to blow-up in size?
%set(0,'DefaultAxesTitleFontSize',16);  % causes title to blow-up in size?
set(0,'DefaultScatterMarker','+');
%set(0,'DefaultScatterSizeData',50^2);
set(0,'DefaultScatterLineWidth',1);
set(0,'DefaultScatterMarkerFaceColor','red');
set(0,'DefaultScatterMarkerEdgeColor','red');
set(0,'DefaultLineColor','black');
set(0,'DefaultLineLineWidth',3);
set(0,'DefaultLineMarkerSize',20);
set(0,'DefaultSurfaceLineWidth',3);
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on');

end