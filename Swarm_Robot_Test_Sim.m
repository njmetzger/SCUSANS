function Swarm_Robot_Test_Sim(Num_Robots,Sim_Time,Sense_Range, AvoidanceRange,DesValue)
% SWARM_ROBOT_TEST_SIM - < Setup and initialization utility for running the
% swarm simulator.>

% Simulation parameters  
NUM_ROBOTS=Num_Robots; 
SIM_TIME=Sim_Time;

SensorRange=Sense_Range;
AvoidRange= AvoidanceRange;
DesiredValue=DesValue;

% Constants 
NUM_SIGNALS_PER_ROBOT=4;
FIELD_WIDTH=350;

VideoFilename='findContour_12_6';

% Setup matlab directory to run in current runFile folder
runFileFunctionName=mfilename;
runFilePath=mfilename('fullpath');
BaseFolderPath = replace(runFilePath,runFileFunctionName,"");
cd(BaseFolderPath);

% If video file already exists, append datetime data to its filename 
if exist([BaseFolderPath,filesep,VideoFilename,'.avi']) == 2
    VideoFilename=[VideoFilename,'_',datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS'),'.avi'];
else
    VideoFilename=[VideoFilename,'.avi'];
end

% Create system with that number of robots 
[h_newsys , simName] = buildSimModel(NUM_ROBOTS,SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Group Robot Behavior Control
for i=1:NUM_ROBOTS
    set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '1');
end

%Runs Simulation, must be the name of the simulink file without the .slx
%extention

% profile on

% Setup waitbar 
h = waitbar(0,'Please wait...');

% Specify sim options and run simulation
% myoptions = simset('SrcWorkspace','current','DstWorkspace','current');
myoptions= simset('Solver','VariableStepAuto');
sim('Swarm_Robot_N',SIM_TIME,myoptions);
simOut.simout=simout;

% Close waitbar
close(h)

est_num_robots=size(simOut.simout.Data,2)/NUM_SIGNALS_PER_ROBOT;

% Throw error if data size mis-matches desired number of robots 
if NUM_ROBOTS~=est_num_robots
    error('Robot number and number of signals defined do not translate');
end

% Resample simulation data to have uniform time step. Use linear
% interpolation to find intermediate values. 
% desiredNumFrames=numel(simOut.simout.Time);
desiredFPS=60;
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
Z=readScalarField(X,Y);
surf(X,Y,Z-3);
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
        addpoints(h_line(k),Robot_Data(k).x(i),Robot_Data(k).y(i));
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


%following line only valid when using the ridge/trench plot from
%Kitts,McDonald, and Neumann paper on cluster adaptive nav primitives: 
global_max_val= 69.15*ones(length(time),1);
desired_contour_plot= DesValue*ones(length(time),1); 
legend_labels= cell(1, (NUM_ROBOTS+1)); 

figure()
hold on 
for i= 1:NUM_ROBOTS
    leg_str1= 'Robot Number  '
    rob_num_legend= num2str(i)
    legend_labels{1,i}= strcat(leg_str1, rob_num_legend);
    plot(time, sensor_value_PI(:,i)) 
    title('Sensor Value Readings')
end 
%plot the concentration at the mean position value 
% plot(time, avg_concentration(:,1));

% plot(time,global_max_val); legend('known maximum value') % plot known max if doing max files
% add the relevant legend label to the legend: 
legend_labels{1,(NUM_ROBOTS+1)}= 'Desired Contour Value';
plot(time,desired_contour_plot,'k--'); legend(legend_labels) % plot contour value if 
hold off 

figure()
hold on
for i=1:NUM_ROBOTS
   leg_str1= 'Robot Number  '
   rob_num_legend= num2str(i)
   %legend_label= strcat(leg_str1, rob_num_legend); 
   plot(Robot_Data(i).x, Robot_Data(i).y);  
   title ('Time History of Robot positions') 
end 

hold off 
% EvaluatePerformance(Robot_Data);
% ADD EVALUATION OF THE ACTUAL MAX FROM SIMULATION 
    % 1) get index of max sensor values 
    % 2) get x,y value at that index 
    % 3) evaluate readScalarField at that x,y and check that it's the same 
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% buildSimModel()
function [h_newsys , simName] = buildSimModel(N, SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue)
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
load_system('Swarm_Robot_Base_2018a')

% Set base robot parameters
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Sensor Range','Value',num2str(SensorRange));
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Avoid Range','Value',num2str(AvoidRange));
set_param('Swarm_Robot_Base_2018a/Robot 1 Behavior/Desired Value','Value',num2str(DesiredValue));

% Construct total system of N robots and link blocks 
for i=1:N
    
    %% Create new "Robot X Behavior" block
    base_model='Swarm_Robot_Base_2018a/Robot 1 Behavior';
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i)

    add_block(base_model,behavior_model);
    
    pos=get_param(behavior_model,'position');
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    
    set_param(behavior_model,'position',pos);
    
    h_behavior{i} = get_param(behavior_model,'PortHandles');
    
    
    %% Create new "Robot X" (robot num) source block
    base_model='Swarm_Robot_Base_2018a/Robot 1';
    new_model=sprintf('Swarm_Robot_N/Robot %i',i)
    
    add_block(base_model,new_model);
    
    pos=get_param(new_model,'position');
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    
    set_param(new_model,'position',pos);
    set_param(new_model,'Value',num2str(i));
    
    h_robot_num{i} = get_param(new_model,'PortHandles');
    
    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_robot_num{i}.Outport(1),h_behavior{i}.Inport(2));
   
   
    %% Create new "Command X" source block
    base_model='Swarm_Robot_Base_2018a/Command 1';
    new_model=sprintf('Swarm_Robot_N/Command %i',i);
    
    add_block(base_model,new_model);
    
    pos=get_param(new_model,'position');
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    
    set_param(new_model,'position',pos);
    set_param(new_model,'Value',get_param(base_model,'Value'));
    
    h_command{i} = get_param(new_model,'PortHandles');
    
    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_command{i}.Outport(1),h_behavior{i}.Inport(3));

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
    add_line('Swarm_Robot_N',h_behavior{i}.Outport(1),h_mux.Inport(i),'autorouting','on');
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
    case 'Set to Random'
        disp('Setting initial conditions to random location on interval')
        
        for i=1:N
            rand_width=FIELD_WIDTH*.9;
            x(i)=rand(1)*(2*rand_width)-rand_width;
            y(i)=rand(1)*(2*rand_width)-rand_width;
            initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Initial Conditions'],'Value',initialCondition{i})
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
        Z=readScalarField(X,Y);
        surf(X,Y,Z);
        view([0 90])
        hold on
        
        for i=1:N
            [x(i),y(i)] = ginput(1)
            z(i)=readScalarField(x(i),y(i));
            plot3(x(i),y(i),z(i)+abs(z(i)*.2),'o','MarkerSize',10,'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','k')
            initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Initial Conditions'],'Value',initialCondition{i})
        end
        close(fig);
end


end





