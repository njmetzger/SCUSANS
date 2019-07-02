function PlotCompositeRobotData(data,ScalarFieldSelection,behavior,field_width)
% <PLOTCOMPOSITEROBOTDATA> - Plots the cartesian time history of all robots
%       in all simulations on a single overhead view plot. 
%
% Syntax:  PlotCompositeRobotData(data)
%
% Inputs:
%    data - simulation output data structure (see sim_runner for definition)

% Outputs:
%    NA - NA

% Extract config data from struct
behavior;
DesiredValue = [];

% Plot robot position history over scalar field surf
figure('name','Composite: Robot Position History')
for i = 1: numel(data)
    
    % Create hsv colormap for robots 
    N = numel(data(i).robot);
    cmap = hsv(N);
    
    for j = 1:N
        plot3(data(i).robot(j).x, data(i).robot(j).y, data(i).robot(j).sensor_value,'LineWidth', 3,'Color',cmap(j,:));
        hold on
    end
end

resolution = 500;
PlotScalarField(gca,ScalarFieldSelection,resolution,DesiredValue,field_width,behavior);

% Only add legend for non-parallel results 
% if numel(data)==1
%     for i=1:numel(data(i).robot)
%         legend_strings{i}=sprintf('Robot %i',i);
%     end
%     legend(legend_strings,'Location','SouthEast')
% end

title(['\fontsize{15} ', 'Robot Position History', char(10) ...
    '\fontsize{12} ', behavior, char(10) ...
    '\fontsize{10} ', sprintf('Number of Robots = %i',N), char(10)],'interpreter','tex');
hold off
end


