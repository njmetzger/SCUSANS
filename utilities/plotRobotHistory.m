%% plotRobotHistory()

function [] = plotRobotHistory(Robot_Data,SimParams,behavior,ScalarFieldSelection,FIELD_WIDTH)
% assignin('base','base_RobotData', Robot_Data)
x_PI= zeros(length(Robot_Data.robot(1).x),SimParams.NUM_ROBOTS);
y_PI= zeros(length(Robot_Data.robot(1).y),SimParams.NUM_ROBOTS);
theta_PI= zeros(length(Robot_Data.robot(1).theta),SimParams.NUM_ROBOTS);
sensor_value_PI = zeros(length(Robot_Data.robot(1).sensor_value),SimParams.NUM_ROBOTS);

for i=1:SimParams.NUM_ROBOTS
    x_PI(:,i) = Robot_Data.robot(i).x;
    y_PI(:,i) = Robot_Data.robot(i).y;
    theta_PI(:,i)= Robot_Data.robot(i).theta;
    sensor_value_PI(:,i)= Robot_Data.robot(i).sensor_value;
end

%Reevaluate X,Y,Z
minx = min(-FIELD_WIDTH,min(min(x_PI)));
miny = min(-FIELD_WIDTH,min(min(y_PI)));
maxx = max(FIELD_WIDTH,max(max(x_PI)));
maxy = max(FIELD_WIDTH,max(max(y_PI)));

ax.XLim=[minx maxx];
ax.YLim=[miny maxy];
res=100;
xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
[X,Y] = meshgrid(xdivs,ydivs);
if ScalarFieldSelection~=5
    Z=readScalarField(X,Y,ScalarFieldSelection);
else
    for i =1:length(X)
        for j = 1:length(Y)
            Z(i,j)=readScalarField(X(i,j),Y(i,j),ScalarFieldSelection);
        end
    end
end



% Determine Average Position of swarm (change to centroid?)
x_PI_ave= zeros(SimParams.NUM_ROBOTS, 1);
y_PI_ave= zeros(SimParams.NUM_ROBOTS, 1);
%avg_concentration= zeros(NUM_ROBOTS,1);

for i= 1:SimParams.NUM_ROBOTS
    x_PI_current= x_PI(:,i);
    x_PI_ave(i,1)= mean(x_PI_current);
    y_PI_current= y_PI(:,i);
    y_PI_ave(i,1)= mean(y_PI_current);
    %avg_concentration(i,1)= readScalarField(x_PI_ave(i,1), y_PI_ave(i,1));
end

% figure()
% plot(x_PI_ave, y_PI_ave)
% Plot the sensor values of each robot to indicate the effectiveness of the
% min/max finding behavior:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time = Robot_Data.out.simout.Time; 
global_max_val= max(max(Z))*ones(length(time),1);
global_min_val= min(min(Z))*ones(length(time),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
desired_contour_plot= SimParams.DESIRED_VALUE*ones(length(time),1);
positive_buffer_plot= desired_contour_plot + SimParams.CONTOUR_BUFFER;
negative_buffer_plot= desired_contour_plot - SimParams.CONTOUR_BUFFER;
%legend_labels= cell(1, (SimParams.NUM_ROBOTS+1));

switch behavior
    case 'Contour Following'
        figure()
        hold on
        for i= 1:SimParams.NUM_ROBOTS
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        %plot the concentration at the mean position value
        % plot(time, avg_concentration(:,1));
        
        % plot(time,global_max_val); legend('known maximum value') % plot known max if doing max files
        % add the relevant legend label to the legend:
        legend_labels{1}= 'Desired Contour Value';
        legend_labels{2}= 'Buffer Limits';
        %legend_labels{1,(NUM_ROBOTS+3)}= 'Negative Buffer Limit';
        ps(1) = plot(time,desired_contour_plot,'k--');  % plot contour value
        ps(2) = plot(time,positive_buffer_plot,'k-.');
        plot(time,negative_buffer_plot,'k-.'); 
        legend(ps,legend_labels);
        xlabel('Time (s)','fontsize',12)
        ylabel('Sensor Value','fontsize',12)
        hold off
        
        
        % EvaluatePerformance(Robot_Data);
        % ADD EVALUATION OF THE ACTUAL MAX FROM SIMULATION
        % 1) get index of max sensor values
        % 2) get x,y value at that index
        % 3) evaluate readScalarField at that x,y and check that it's the same
        
    case 'Find Min'
        figure()
        hold on
        for i= 1:SimParams.NUM_ROBOTS
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        % add reference for global min:
        legend_labels{1}= 'Known Global Minimum';
        ps = plot(time,global_min_val,'k--'); legend(ps, legend_labels) ;% plot minimum value
        xlabel('Time (s)','fontsize',12)
        ylabel('Sensor Value','fontsize',12)
    case 'Find Max'
        % plot the individual robot concentrations:
        figure()
        hold on
        for i= 1:SimParams.NUM_ROBOTS
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        % add reference for global max:
        legend_labels{1}= 'Known Global Maximum';
        ps= plot(time,global_max_val,'k--'); legend(ps, legend_labels); % plot maximum value
        xlabel('Time (s)')
        ylabel('Sensor Value')
    case 'Go To'
        figure()
        hold on
        xs = zeros(1,SimParams.NUM_ROBOTS);
        ys = zeros(1,SimParams.NUM_ROBOTS);
        for i=1:SimParams.NUM_ROBOTS
            plot(Robot_Data.robot(i).x, Robot_Data.robot(i).y,'LineWidth',2);
            ps(1) = plot(Robot_Data.robot(i).x(1), Robot_Data.robot(i).y(1), 'kx');
            ps(2) = plot(Robot_Data.robot(i).x(end), Robot_Data.robot(i).y(end), 'ko');
            xs(i) = Robot_Data.robot(i).x(end);
            ys(i) = Robot_Data.robot(i).y(end);
        end
        x = sum(xs)/SimParams.NUM_ROBOTS;
        y = sum(ys)/SimParams.NUM_ROBOTS;
        ps(3) = plot(x,y,'k*','MarkerSize',18);
        ps(4) = plot(SimParams.GoTo_Coords(1),SimParams.GoTo_Coords(2), 'ks','MarkerSize',18);
        title ('Time History of Robot Positions','fontsize',12), xlabel('X (m)','fontsize',12), ylabel('Y (m)','fontsize',12),
        legend(ps, 'Starting Position', 'Ending Position', 'Swarm Average End Position', 'Desired Coordinates')
        hold off
    case 'Incompatible'
        disp('You have selected an incompatible pair of behaviors, such as selecting multiple of the FindMin/FindMax/FindContour behaviors. Plots could not be generated.')
        
    otherwise
        disp('Selected Behavior(s) do not currently have a post-processing plot available.')
end

figure()
hold on
for i=1:SimParams.NUM_ROBOTS
    plot3(Robot_Data.robot(i).x, Robot_Data.robot(i).y,Robot_Data.robot(i).sensor_value,'LineWidth',2);
end
contour3(X,Y,Z,15)
view(-45,45)
title ('Time History of Robot Positions','fontsize',12), xlabel('X (m)','fontsize',12), ylabel('Y (m)','fontsize',12),zlabel('Sensor Value','fontsize',12)
hold off
end