function [h] = circle(x,y,r,varargin)
%% This function plots a circle centred at location x,y with radius r.
% Use 'Color' and 'FillColor' to specify different fill and edge colours.
% Ref: Mathworks
% Author: Hannah Sheahan, sheahan.hannah@gmail.com

%% default settings
while ~isempty(varargin)
    switch varargin{1}
        case 'Color'
            colour = varargin{2};
        case 'FillColor'
            fill_colour = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
    varargin(1:2) = [];
end
if exist('colour','var')==0
    colour = 'k';
end

%% plot circle
th = 0:pi/50:2*pi;
xunit = r*cos(th) + x;
yunit = r*sin(th) + y;

if exist('fill_colour','var')
    fill(xunit, yunit, fill_colour); hold on;     % fill circle
end
h = plot(xunit, yunit,'Color',colour,'LineWidth',1.5); hold on;   % plot circle edge on top

end

