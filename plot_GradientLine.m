function plot_GradientLine(x,y,c,varargin)
%% FUNCTION: Plots a line with a gradated color.
% INPUTS:   x = x coordinates
%           y = y coordinates
%           c = colors
% OUTPUTS:  N/A
% NOTES:    Can pass other variables e.g. {'LineWidth',3}
% ISSUES:   N/A
% REFS:     http://blogs.mathworks.com/videos/2014/08/12/coloring-a-line-based-on-height-gradient-or-some-other-value-in-matlab/
% AUTHOR:   Daniel McNamee, daniel.c.mcnamee@gmail.com

%% otherwise
while ~isempty(varargin)
    switch lower(varargin{1})
        case 'z'
            z = varargin{2};
        otherwise
            error(['Unexpected option: ' varargin{1}])
    end
      varargin(1:2) = [];
end
if isempty(z)
    z = zeros(1,numel(x));
end

surface('XData',[x; x],...
        'YData',[y; y],...
        'ZData',[z; z],...
        'CData',[c; c],...
        'FaceColor','no',...
        'EdgeColor','interp',...
        'Marker','o',...
        varargin{:});

end