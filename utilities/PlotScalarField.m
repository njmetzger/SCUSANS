%% PlotScalarField
function [s,Z] = PlotScalarField(ax,ScalarFieldSelection,resolution,DesiredValue,field_width,behavior)

hold(ax,'on')
ax.XLim = [-field_width field_width];
ax.YLim = [-field_width field_width];
divs = linspace(-field_width*1.5, field_width*1.5, resolution);  % 50pct extra overlap to allow for panning
[X,Y] = meshgrid(divs,divs);
if ScalarFieldSelection ~=5
    Z=readScalarField(X,Y,ScalarFieldSelection);
else
    for i = 1:length(X)
        for j = 1:length(Y)
            Z(i,j) = readScalarField(X(i,j),Y(i,j),ScalarFieldSelection);
        end
    end
end
s = surf(ax,X,Y,Z-1,'EdgeColor','none');  % Want surface to be slightly below robot lines
view([0 90])

% Highlight contour if specified
if (exist('DesiredValue','var') || ~isempty(DesiredValue) ) && strcmp(behavior,'Contour Following')
    hold on
    [M,c] = contour3(ax,X,Y,Z,[DesiredValue DesiredValue],'ShowText','on','LineWidth',3,'LineColor',[.1 .1 .1]);
    [M,c] = contour3(ax,X,Y,Z,[DesiredValue DesiredValue],'ShowText','on','LineWidth',2);
    hold off
end
end