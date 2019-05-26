%% plotRobotHistory()
function [] = plotRobotHistory(data)

Robot_Data = data.robot;
NUM_ROBOTS = data.cfg.Nrobots;
CONTOUR_BUFFER = data.cfg.CONTOUR_BUFFER;

% assignin('base','base_RobotData', Robot_Data)
x_PI= zeros(length(Robot_Data(1).x),NUM_ROBOTS);
y_PI= zeros(length(Robot_Data(1).y),NUM_ROBOTS);
theta_PI= zeros(length(Robot_Data(1).theta),NUM_ROBOTS);
sensor_value_PI = zeros(length(Robot_Data(1).sensor_value),NUM_ROBOTS);

for i=1:NUM_ROBOTS
    x_PI(:,i) = Robot_Data(i).x;
    y_PI(:,i) = Robot_Data(i).y;
    theta_PI(:,i)= Robot_Data(i).theta;
    sensor_value_PI(:,i)= Robot_Data(i).sensor_value;
end

%Reevaluate X,Y,Z
minx = min(min(min(X)),min(min(x_PI)));
miny = min(min(min(Y)),min(min(y_PI)));
maxx = max(max(max(X)),max(max(x_PI)));
maxy = max(max(max(Y)),max(max(y_PI)));

ax.XLim=[minx maxx];
ax.YLim=[miny maxy];
res=100;
xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
[X,Y] = meshgrid(xdivs,ydivs);
Z=readScalarField(X,Y,ScalarFieldSelection);

% Determine Average Position of swarm (change to centroid?)
x_PI_ave= zeros(NUM_ROBOTS, 1);
y_PI_ave= zeros(NUM_ROBOTS, 1);

for i= 1:NUM_ROBOTS
    x_PI_current= x_PI(i,:);
    x_PI_ave(i,1)= mean(x_PI_current);
    y_PI_current= y_PI(i,:);
    y_PI_ave(i,1)= mean(y_PI_current);
end

% figure()
% plot(x_PI_ave, y_PI_ave)
% Plot the sensor values of each robot to indicate the effectiveness of the
% min/max finding behavior:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global_max_val= max(max(Z))*ones(length(time),1);
global_min_val= min(min(Z))*ones(length(time),1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
desired_contour_plot= DesValue*ones(length(time),1);
positive_buffer_plot= desired_contour_plot + CONTOUR_BUFFER;
negative_buffer_plot= desired_contour_plot - CONTOUR_BUFFER;
legend_labels= cell(1, (NUM_ROBOTS+1));

switch behavior
    case 'Contour Following'
        figure()
        hold on
        for i= 1:NUM_ROBOTS
            leg_str1= 'Robot Number  ';
            rob_num_legend= num2str(i);
            legend_labels{1,i}= strcat(leg_str1, rob_num_legend);
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        %plot the concentration at the mean position value
        % plot(time, avg_concentration(:,1));
        
        % plot(time,global_max_val); legend('known maximum value') % plot known max if doing max files
        % add the relevant legend label to the legend:
        legend_labels{1,(NUM_ROBOTS+1)}= 'Desired Contour Value';
        legend_labels{1,(NUM_ROBOTS+2)}= 'Buffer Limits';
        %legend_labels{1,(NUM_ROBOTS+3)}= 'Negative Buffer Limit';
        plot(time,desired_contour_plot,'k--');  % plot contour value
        plot(time,positive_buffer_plot,'k-.');
        plot(time,negative_buffer_plot,'k-.'); legend(legend_labels);
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
        for i= 1:NUM_ROBOTS
            leg_str1= 'Robot Number  ';
            rob_num_legend= num2str(i);
            legend_labels{1,i}= strcat(leg_str1, rob_num_legend);
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        % add reference for global min:
        legend_labels{1,(NUM_ROBOTS+1)}= 'Known Global Minimum';
        plot(time,global_min_val,'k--'); legend(legend_labels) ;% plot minimum value
        xlabel('Time (s)','fontsize',12)
        ylabel('Sensor Value','fontsize',12)
    case 'Find Max'
        % plot the individual robot concentrations:
        figure()
        hold on
        for i= 1:NUM_ROBOTS
            leg_str1= 'Robot Number  ';
            rob_num_legend= num2str(i);
            legend_labels{1,i}= strcat(leg_str1, rob_num_legend);
            plot(time, sensor_value_PI(:,i))
        end
        title('Sensor Value Readings','fontsize',12)
        % add reference for global max:
        legend_labels{1,(NUM_ROBOTS+1)}= 'Known Global Maximum';
        plot(time,global_max_val,'k--'); legend(legend_labels); % plot maximum value
        xlabel('Time (s)')
        ylabel('Sensor Value')
    case 'Go To'
        figure()
        hold on
        robot_legend = {}; 
        xs = zeros(1,NUM_ROBOTS); 
        ys = zeros(1,NUM_ROBOTS);
        for i=1:NUM_ROBOTS
            leg_str1= 'Robot Number  ';
            rob_num_legend= num2str(i);
            robot_legend{i} = strcat(leg_str1,leg_str1); 
            %legend_label= strcat(leg_str1, rob_num_legend);
            ps(i) = plot(Robot_Data(i).x, Robot_Data(i).y,'LineWidth',2);
            plot(Robot_Data(i).x(1), Robot_Data(i).y(1), 'kx');
            plot(Robot_Data(i).x(end), Robot_Data(i).y(end), 'ko')
            xs(i) = Robot_Data(i).x(end); 
            ys(i) = Robot_Data(i).y(end);
        end
        x = sum(xs)/NUM_ROBOTS;
        y = sum(ys)/NUM_ROBOTS;
        ps(length(ps)+1) = plot(x,y,'k*','MarkerSize',18);
        ps(length(ps)+1) = plot(GoTo(1),GoTo(2), 'ks','MarkerSize',18);
        title ('Time History of Robot Positions','fontsize',12), xlabel('X (m)','fontsize',12), ylabel('Y (m)','fontsize',12),
        hold off
    case 'Incompatible'
        disp('You have selected an incompatible pair of behaviors, such as selecting multiple of the FindMin/FindMax/FindContour behaviors. Plots could not be generated.')
        
    otherwise
        disp('Selected Behavior(s) do not currently have a post-processing plot available.')
end

figure()
hold on
for i=1:NUM_ROBOTS
    leg_str1= 'Robot Number  ';
    rob_num_legend= num2str(i);
    %legend_label= strcat(leg_str1, rob_num_legend);
    plot3(Robot_Data(i).x, Robot_Data(i).y,Robot_Data(i).sensor_value,'LineWidth',2);
end
contour3(X,Y,Z)
view(-45,45)
%surf(X,Y,Z)
title ('Time History of Robot Positions','fontsize',12), xlabel('X (m)','fontsize',12), ylabel('Y (m)','fontsize',12),zlabel('Sensor Value','fontsize',12)
hold off
end