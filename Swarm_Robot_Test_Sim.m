function Swarm_Robot_Test_Sim(NUM_ROBOTS,Sim_Time,Sense_Range,AvoidanceRange,...
    DesValue,CONTOUR_BUFFER,ScalarFieldSelection,behavior,x_init,y_init,...
    radius_init,isExp, robots,base)
% SWARM_ROBOT_TEST_SIM - < Setup and initialization utility for running the
% swarm simulator.>

% Simulation parameters
SIM_TIME=Sim_Time;

SensorRange=Sense_Range;
AvoidRange= AvoidanceRange;
DesiredValue=DesValue;
SCALAR_FIELD= ScalarFieldSelection

% Constants
NUM_SIGNALS_PER_ROBOT=4;
FIELD_WIDTH=300;

VideoFilename='findContour_12_6';

% Setup matlab directory to run in current runFile folder
runFileFunctionName=mfilename;
runFilePath=mfilename('fullpath');
BaseFolderPath = replace(runFilePath,runFileFunctionName,string(''));
cd(BaseFolderPath);

% If video file already exists, append datetime data to its filename
if exist([BaseFolderPath,filesep,VideoFilename,'.avi']) == 2
    VideoFilename=[VideoFilename,'_',datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS'),'.avi'];
else
    VideoFilename=[VideoFilename,'.avi'];
end

% Create system with that number of robots
[h_newsys , simName] = buildSimModel(NUM_ROBOTS,SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue,CONTOUR_BUFFER,ScalarFieldSelection,x_init,y_init,radius_init,isExp,robots,base);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Group Robot Behavior Control

%Runs Simulation, must be the name of the simulink file without the .slx
%extention

% profile on

% Setup waitbar
%h = waitbar(0,'Please wait...');

% Specify sim options and run simulation
% myoptions = simset('SrcWorkspace','current','DstWorkspace','current');
if isExp
    myoptions = simset('Solver', 'FixedStepDiscrete', 'FixedStep', '0.1');
    sim('Swarm_Robot_N',SIM_TIME,myoptions)
else
    myoptions = simset('Solver', 'VariableStepAuto');
    sim('Swarm_Robot_N',SIM_TIME,myoptions)
end
simOut.simout=simout;

% Close waitbar
% close(h)

est_num_robots=size(simOut.simout.Data,2)/NUM_SIGNALS_PER_ROBOT;

% Throw error if data size mis-matches desired number of robots
if NUM_ROBOTS~=est_num_robots
    error('Robot number and number of signals defined do not translate');
end

% Resample simulation data to have uniform time step. Use linear
% interpolation to find intermediate values.
% desiredNumFrames=numel(simOut.simout.Time);
desiredFPS=10;
desiredNumFrames=desiredFPS*SIM_TIME;
uniform_time=linspace(0,max(simOut.simout.Time),desiredNumFrames);
resamp_data=resample(simOut.simout,uniform_time);
simOut.simout=resamp_data;

% Extract individual robot data and organize into structure array
for i=1:NUM_ROBOTS
    Robot_Data(i).x=simOut.simout.Data(:,i*4-3);
    Robot_Data(i).y=simOut.simout.Data(:,i*4-2);
    Robot_Data(i).theta=simOut.simout.Data(:,i*4-1);
    Robot_Data(i).sensor_value=simOut.simout.Data(:,i*4);
end

% Extract time vector
time=simOut.simout.time;
dt=diff(time);

%% Plots robots positions

% Setup videowriter
v = VideoWriter(VideoFilename,'Motion JPEG AVI')
% v.FrameRate = floor(numel(simOut.simout.time)/SIM_TIME);
v.FrameRate=desiredFPS;
v.Quality=75;
open(v)
F(numel(Robot_Data)) = struct('cdata',[],'colormap',[]);

% Create unique color map for each robot
cmap = hsv(numel(Robot_Data));

% First plot scalar field
figure(1)
ax=gca;
ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
res=100;
xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
[X,Y] = meshgrid(xdivs,ydivs);
Z=readScalarField(X,Y,ScalarFieldSelection);
surf(X,Y,Z-3);  % why is this surfed at z-3??
view([0 90])
axis([-FIELD_WIDTH FIELD_WIDTH -FIELD_WIDTH FIELD_WIDTH])
hold on
[M,c] =contour3(X,Y,Z,[DesiredValue DesiredValue],'ShowText','on');
c.LineWidth = 3;
hold off

% Create animated lines for each robot
for i=1:numel(Robot_Data)
    h_line(i)=animatedline('Marker','o','MarkerFaceColor',cmap(i,:),'LineWidth',3,'MaximumNumPoints',1);
end

% Animate the simulation results
for i=1:size(simOut.simout.Data,1)-1
    
    % Update position points for each robot
    for k=1:NUM_ROBOTS
        addpoints(h_line(k),Robot_Data(k).x(i),Robot_Data(k).y(i),(Robot_Data(k).sensor_value(i)+50));  % can we add in the z-value to the plot here?
    end
    
    drawnow limitrate
    % pause(dt(i));
    F(i) = getframe(gcf);
    writeVideo(v,F(i))
end

% fig =figure
% movie(fig,F,60)
% for i=1:NUM_ROBOTS
%     writeVideo(v,F(i));
% end
close(v)

% profile viewer
% Peformance_eval
% % Get individual robot position history from Robot_Data
%
% %initialize variables
%
time=simOut.simout.time;

%% Plot time history of robots
plotRobotHistory(Robot_Data, NUM_ROBOTS,time,CONTOUR_BUFFER,DesValue,behavior);

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% buildSimModel()
function [h_newsys , simName] = buildSimModel(N, SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue, CONTOUR_BUFFER,ScalarFieldSelection,x_init,y_init,radius_init,isExp,robots,base)
% Script for generating simulink model for N robots

% Define system name
simName='Swarm_Robot_N';

% Default vertical spacing of blocks
vert_spacing = -150;

% If a system is already open with the same name, close it without saving
if exist('Swarm_Robot_N')
    close_system('Swarm_Robot_N',0)
end

% Create new blank simulink model
h_newsys = new_system('Swarm_Robot_N');
open_system(h_newsys);

% Load the template "Robot Behavior" model
load_system(base)

% Set base robot parameters
set_param(strcat(base,'/Robot 1 Behavior/Sensor Range'),'Value',num2str(SensorRange));
set_param(strcat(base,'/Robot 1 Behavior/Avoid Range'),'Value',num2str(AvoidRange));
set_param(strcat(base,'/Robot 1 Behavior/Desired Value'),'Value',num2str(DesiredValue));
set_param(strcat(base,'/Robot 1 Behavior/Contour Buffer'), 'Value',num2str(CONTOUR_BUFFER));
set_param(strcat(base,'/Robot 1 SimResponse/ScalarFieldSelection'),'Value',num2str(ScalarFieldSelection));

% Construct total system of N robots and link blocks
if isExp
    rs = string('{');
    for j = 1:length(robots)
        if j ==1
            rs = rs+ sprintf(string('''%s'''),robots(j));
        else
            rs = rs+ sprintf(',''%s''', robots(j));
        end
    end
    rs = rs + '}';
    add_block(strcat(base,'/Loop_Pacer'), 'Swarm_Robot_N/Loop_Pacer')
    add_block(strcat(base,'/Optitrack'), 'Swarm_Robot_N/Optitrack')
    set_param('Swarm_Robot_N/Optitrack', 'rigidBodyNames', rs)
    OT_handles= get_param('Swarm_Robot_N/Optitrack', 'PortHandles');
end
for i=1:N
    %% Add blocks for experiment if needed
    if isExp
        %Create Signal Reshape for each robot
        base_model_SR = strcat(base,'/Signal_Reshape');
        new_model_SR = sprintf('Swarm_Robot_N/Signal_Reshape_%d',i);
        addBlockAndSpace(base_model_SR, new_model_SR,vert_spacing,0,i )
        RS_handles= get_param(new_model_SR, 'PortHandles');
        add_line('Swarm_Robot_N',OT_handles.Outport(i),RS_handles.Inport(1));

        %Create Signal Selector for each robot
        base_model_ss = strcat(base,'/Signal_Select');
        new_model_ss = sprintf('Swarm_Robot_N/Signal_Select_%d',i);
        addBlockAndSpace(base_model_ss, new_model_ss,vert_spacing,0,i)
        SS_handles{i}= get_param(new_model_ss, 'PortHandles');
        add_line('Swarm_Robot_N',RS_handles.Outport(1),SS_handles{i}.Inport(1));
    end


    %% Create new "Robot X Behavior" block
    base_model=strcat(base,'/Robot 1 Behavior');
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i)
    addBlockAndSpace(base_model, behavior_model,vert_spacing, 0, i )

    h_behavior{i} = get_param(behavior_model,'PortHandles');

    %% Create new "Robot X Response" block
    if isExp
        base_model=strcat(base,'/Robot 1 ExpResponse');
        response_model=sprintf('Swarm_Robot_N/Robot %i ExpResponse',i);
        addBlockAndSpace(base_model, response_model,vert_spacing, 0, i )

        b_behavior{i} = get_param(response_model,'PortHandles');
        add_line('Swarm_Robot_N', SS_handles{i}.Outport(1), b_behavior{i}.Inport(2));
        pos=get_param(response_model,'position');
    else
        base_model=strcat(base,'/Robot 1 SimResponse');
        response_model=sprintf('Swarm_Robot_N/Robot %i SimResponse',i);
        addBlockAndSpace(base_model, response_model,vert_spacing, 0, i )

        b_behavior{i} = get_param(response_model,'PortHandles');
        pos=get_param(response_model,'position');

    end

    %% Create new "Robot X" (robot num) source block
    base_model=strcat(base,'/Robot 1');
    new_model=sprintf('Swarm_Robot_N/Robot %i',i)
    addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )

    set_param(new_model,'Value',num2str(i));

    h_robot_num{i} = get_param(new_model,'PortHandles');

    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_robot_num{i}.Outport(1),h_behavior{i}.Inport(2));


    %% Create new "Command X" source block
    base_model=strcat(base,'/Command 1');
    new_model=sprintf('Swarm_Robot_N/Command %i',i);
    addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )

    set_param(new_model,'Value',get_param(base_model,'Value'));

    h_command{i} = get_param(new_model,'PortHandles');

    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_command{i}.Outport(1),h_behavior{i}.Inport(3));

    % Connect robot behavior block to robot response block
    add_line('Swarm_Robot_N', h_behavior{i}.Outport(1), b_behavior{i}.Inport(1));

    %Add communication blocks for experiment robots
    if isExp
        new_model = sprintf('Swarm_Robot_N/Robot %d ExpResponse/Robot_command', i);
        set_param(new_model, 'robotName', robots(i))
        CB_handles = get_param(new_model, 'PortHandles');
    end

end

% Add Mux block for all robot signals
add_block('simulink/Signal Routing/Mux','Swarm_Robot_N/Mux');

set_param('Swarm_Robot_N/Mux','Inputs',num2str(N))
set_param('Swarm_Robot_N/Mux','Orientation','down')

%[left top right bottom]
last_pos=pos;
pos=last_pos;
pos(1)=-1167 ;
pos(2)=last_pos(2)-vert_spacing;
pos(3)=-969;
pos(4)=pos(2)+15;
set_param('Swarm_Robot_N/Mux','position',pos)

h_mux = get_param('Swarm_Robot_N/Mux','PortHandles');

% Connect mux inputs and outputs
for i=1:N
    add_line('Swarm_Robot_N',b_behavior{i}.Outport(1),h_mux.Inport(i),'autorouting','on');
    add_line('Swarm_Robot_N',h_mux.Outport(1),h_behavior{i}.Inport(1),'autorouting','on');
end

%% Create new "To Workspace" block
% Add Mux block for all robot signals
add_block('simulink/Sinks/To Workspace','Swarm_Robot_N/SimOut_Data');

%[left top right bottom]
last_pos=pos;
pos=last_pos;
pos(1)=last_pos(1)+300 ;
pos(2)=last_pos(2)+46;
pos(3)=last_pos(3)+271;
pos(4)=last_pos(4)+129;
set_param('Swarm_Robot_N/SimOut_Data','position',pos)

h_simout = get_param('Swarm_Robot_N/SimOut_Data','PortHandles');

h_command{i} = get_param(new_model,'PortHandles');

% Connect robot num to behavior block
add_line('Swarm_Robot_N',h_mux.Outport(1),h_simout.Inport(1),'autorouting','on');

% set_param('Swarm_Robot_N/SimOut_Data','SaveFormat','Structure With Time')

% %% Add a waitbar system
% load_system('waitbar_system')
%
% add_block('waitbar_system/Waitbar_Timer','Swarm_Robot_N/Waitbar_Timer');
% set_param('waitbar_system/Waitbar_Timer/End_Time','Value',num2str(SIM_TIME));

%% Define robot initial conditions
answer = questdlg('Select initial condition option?', ...
    'Initial Condition Options', ...
    'Use Default','Set to Random','Select Manually','Use Default');
% Handle response
switch answer
    case 'Use Default'
        disp('Using default initial conditions. NOTE: Currently defaulting to RANDOM')
        % set initial conditions as center point and circle:
        center_point_x= x_init;
        center_point_y= y_init;
        radius_circle= radius_init; 
        theta_offset= 2*pi/N;
        for i= 1:N
            x(i) = center_point_x + radius_circle*cos(i*theta_offset);
            y(i)= center_point_y + radius_circle*sin(i*theta_offset);
            initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Initial Conditions'],'Value',initialCondition{i})
        end
        
    case 'Set to Random'
        disp('Setting initial conditions to random location on interval')
        
        for i=1:N
            rand_width=FIELD_WIDTH*.9;
            x(i)=rand(1)*(2*rand_width)-rand_width;
            y(i)=rand(1)*(2*rand_width)-rand_width;
            if ~isExp
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
        end
    case 'Select Manually'
        disp('Opening GUI interface for selection')
        fig=figure
        
        % First plot scalar field
        ax=gca;
        ax.XLim=[-FIELD_WIDTH FIELD_WIDTH];
        ax.YLim=[-FIELD_WIDTH FIELD_WIDTH];
        cmap = hsv(N);
        title('Click to select robot initial positions')
        
        res=100;
        xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
        ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
        [X,Y] = meshgrid(xdivs,ydivs);
        Z=readScalarField(X,Y,ScalarFieldSelection);
        surf(X,Y,Z);
        view([0 90])
        hold on
        
        for i=1:N
            [x(i),y(i)] = ginput(1)
            z(i)=readScalarField(x(i),y(i),ScalarFieldSelection);
            plot3(x(i),y(i),z(i)+abs(z(i)*.2),'o','MarkerSize',10,'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','k')
            if ~isExp
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
        end
        close(fig);
end


end

%% plotRobotHistory()

function [] = plotRobotHistory(Robot_Data, NUM_ROBOTS,time,CONTOUR_BUFFER,DesValue,behavior)

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

% Determine Average Position of swarm (change to centroid?)
x_PI_ave= zeros(NUM_ROBOTS, 1);
y_PI_ave= zeros(NUM_ROBOTS, 1);
%avg_concentration= zeros(NUM_ROBOTS,1);

for i= 1:NUM_ROBOTS
    x_PI_current= x_PI(i,:);
    x_PI_ave(i,1)= mean(x_PI_current);
    y_PI_current= y_PI(i,:);
    y_PI_ave(i,1)= mean(y_PI_current);
    %avg_concentration(i,1)= readScalarField(x_PI_ave(i,1), y_PI_ave(i,1));
end

% figure()
% plot(x_PI_ave, y_PI_ave)
% Plot the sensor values of each robot to indicate the effectiveness of the
% min/max finding behavior:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%following lines only valid when using the ridge/trench plot from
%Kitts,McDonald, and Neumann paper on cluster adaptive nav primitives:
global_max_val= 69.15*ones(length(time),1);
global_min_val= -29.5*ones(length(time),1);
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
            title('Sensor Value Readings')
        end
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
            title('Sensor Value Readings')
        end
        % add reference for global min:
        legend_labels{1,(NUM_ROBOTS+1)}= 'Known Global Minimum';
        plot(time,global_min_val,'k--'); legend(legend_labels) ;% plot minimum value
        
    case 'Find Max'
        % plot the individual robot concentrations:
        figure()
        hold on
        for i= 1:NUM_ROBOTS
            leg_str1= 'Robot Number  ';
            rob_num_legend= num2str(i);
            legend_labels{1,i}= strcat(leg_str1, rob_num_legend);
            plot(time, sensor_value_PI(:,i))
            title('Sensor Value Readings')
        end
        % add reference for global max:
        legend_labels{1,(NUM_ROBOTS+1)}= 'Known Global Maximum';
        plot(time,global_max_val,'k--'); legend(legend_labels); % plot maximum value
        
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
    plot(Robot_Data(i).x, Robot_Data(i).y);
    title ('Time History of Robot positions')
end

hold off
end

function [] = addBlockAndSpace(base_model, new_model,vert_spacing, hor_spacing, i )
    add_block(base_model, new_model)
    pos=get_param(new_model,'position');
    pos(1)=pos(1)-hor_spacing; 
    pos(3)=pos(3)-hor_spacing;
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    set_param(new_model,'position',pos);
end


