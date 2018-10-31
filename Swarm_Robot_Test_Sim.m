function Swarm_Robot_Test_Sim()
% SWARM_ROBOT_TEST_SIM - < Setup and initialization utility for running the
% swarm simulator.>
%setup
clc
clear
close all
% Simulation parameters
isExp= true;
if isExp
    robots = ["pink", "blue","red"];
    NUM_ROBOTS = length(robots);
else
    NUM_ROBOTS=5; 
    robots = [];
end

SIM_TIME=10;

SensorRange=1.5;
AvoidRange=.6;
DesiredValue=2;

% Constants 
NUM_SIGNALS_PER_ROBOT=4;
FIELD_WIDTH=3;

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
[h_newsys , simName] = buildSimModel(NUM_ROBOTS,SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue,isExp,robots);

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
sim('Swarm_Robot_N',SIM_TIME)
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

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% buildSimModel()
function [h_newsys , simName] = buildSimModel(N, SIM_TIME, FIELD_WIDTH, SensorRange, AvoidRange, DesiredValue,isExp, robots)
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
load_system('Swarm_Robot_Base')

% Set base robot parameters
set_param('Swarm_Robot_Base/Robot 1 Behavior/Sensor Range','Value',num2str(SensorRange));
set_param('Swarm_Robot_Base/Robot 1 Behavior/Avoid Range','Value',num2str(AvoidRange));
%set_param('Swarm_Robot_Base/Robot 1 Behavior/Desired Value','Value',num2str(DesiredValue));

% Construct total system of N robots and link blocks 
if isExp
    rs = "{";
    for j = 1:length(robots)
        if j ==1
            rs = rs+ sprintf("'%s'",robots(j));
        else
            rs = rs+ sprintf(",'%s'", robots(j));
        end
    end
    rs = rs + "}";
    add_block('Swarm_Robot_Base/Loop_Pacer', 'Swarm_Robot_N/Loop_Pacer')
    add_block('Swarm_Robot_Base/Optitrack', 'Swarm_Robot_N/Optitrack')
    set_param('Swarm_Robot_N/Optitrack', 'rigidBodyNames', rs)
    OT_handles= get_param('Swarm_Robot_N/Optitrack', 'PortHandles');
end
for i=1:N
    %% Add blocks for experiment if needed
    if isExp
        %Create Signal Reshape for each robot
        base_model_SR = 'Swarm_Robot_Base/Signal_Reshape';
        new_model_SR = sprintf('Swarm_Robot_N/Signal_Reshape_%d',i);
        addBlockAndSpace(base_model_SR, new_model_SR,vert_spacing,0,i )
        RS_handles= get_param(new_model_SR, 'PortHandles');
        add_line('Swarm_Robot_N',OT_handles.Outport(i),RS_handles.Inport(1));

        %Create Signal Selector for each robot
        base_model_ss = 'Swarm_Robot_Base/Signal_Select';
        new_model_ss = sprintf('Swarm_Robot_N/Signal_Select_%d',i);
        addBlockAndSpace(base_model_ss, new_model_ss,vert_spacing,0,i)
        SS_handles{i}= get_param(new_model_ss, 'PortHandles');
        add_line('Swarm_Robot_N',RS_handles.Outport(1),SS_handles{i}.Inport(1));
    end
    
    
    %% Create new "Robot X Behavior" block
    base_model='Swarm_Robot_Base/Robot 1 Behavior';
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i)
    addBlockAndSpace(base_model, behavior_model,vert_spacing, 0, i )
    
    h_behavior{i} = get_param(behavior_model,'PortHandles');
    
    %% Create new "Robot X Response" block
    if isExp
        base_model='Swarm_Robot_Base/Robot 1 ExpResponse';
        response_model=sprintf('Swarm_Robot_N/Robot %i ExpResponse',i);
        addBlockAndSpace(base_model, response_model,vert_spacing, 0, i )

        b_behavior{i} = get_param(response_model,'PortHandles');
    else
        base_model='Swarm_Robot_Base/Robot 1 SimResponse';
        response_model=sprintf('Swarm_Robot_N/Robot %i SimResponse',i);
        addBlockAndSpace(base_model, response_model,vert_spacing, 0, i )

        b_behavior{i} = get_param(response_model,'PortHandles');
        pos=get_param(response_model,'position');
    
    end
    
    %% Create new "Robot X" (robot num) source block
    base_model='Swarm_Robot_Base/Robot 1';
    new_model=sprintf('Swarm_Robot_N/Robot %i',i)
    addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )
    
    set_param(new_model,'Value',num2str(i));
    
    h_robot_num{i} = get_param(new_model,'PortHandles');
    
    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_robot_num{i}.Outport(1),h_behavior{i}.Inport(2));
   
   
    %% Create new "Command X" source block
    base_model='Swarm_Robot_Base/Command 1';
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
        %Create Robot velocity Demuxers
        base_model = 'Swarm_Robot_Base/Demux_velocity';
        new_model = sprintf('Swarm_Robot_N/Demux_velocity_%d', i);
        addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )
        VD_handle = get_param(new_model, 'PortHandles');
        %Add communication Blocks
        base_model = 'Swarm_Robot_Base/Robot_command';
        new_model = sprintf('Swarm_Robot_N/Robot_command_%s', robots(i));
        hor_spacing = -50;
        addBlockAndSpace(base_model, new_model,vert_spacing, hor_spacing, i )
        set_param(new_model, 'robotName', robots(i))
        CB_handles = get_param(new_model, 'PortHandles');

        %Add signal bus block
        base_model = 'Swarm_Robot_Base/Bus_signal';
        new_model = sprintf('Swarm_Robot_N/Bus_signal_%d', i);
        addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )
        BB_handles = get_param(new_model, 'PortHandles');
        pos=get_param(new_model,'position'); 
        %add lines
        add_line('Swarm_Robot_N', b_behavior{i}.Outport(1), VD_handle.Inport(1))
        for j = 1:3
            add_line('Swarm_Robot_N',VD_handle.Outport(j),CB_handles.Inport(j));
        end
        for k = 1:4
            add_line('Swarm_Robot_N',CB_handles.Outport(k),BB_handles.Inport(k));
        end
    end

end

% Add Mux block for all robot signals
add_block('simulink/Signal Routing/Mux','Swarm_Robot_N/Mux');

set_param('Swarm_Robot_N/Mux','Inputs',num2str(N))
set_param('Swarm_Robot_N/Mux','Orientation','down')

%[left top right bottom]
pos(1)=pos(1)+100 ;
pos(2)=pos(2)-vert_spacing;
pos(3)=pos(1)+200;
pos(4)=pos(2)+15;
set_param('Swarm_Robot_N/Mux','position',pos)

h_mux = get_param('Swarm_Robot_N/Mux','PortHandles');

% Connect mux inputs and outputs 
if isExp
    for i=1:N
        add_line('Swarm_Robot_N',SS_handles{i}.Outport(1),h_mux.Inport(i),'autorouting','on');
        add_line('Swarm_Robot_N',SS_handles{i}.Outport(1),h_behavior{i}.Inport(1),'autorouting','on');
    end
else
    for i=1:N
        add_line('Swarm_Robot_N',b_behavior{i}.Outport(1),h_mux.Inport(i),'autorouting','on');
        add_line('Swarm_Robot_N',h_mux.Outport(1),h_behavior{i}.Inport(1),'autorouting','on');
    end
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
load_system('waitbar_system')

add_block('waitbar_system/Waitbar_Timer','Swarm_Robot_N/Waitbar_Timer');
set_param('waitbar_system/Waitbar_Timer/End_Time','Value',num2str(SIM_TIME));

%% Define robot initial conditions
if ~isExp
    answer = questdlg('Select initial condition option?', ...
        'Initial Condition Options', ...
        'Use Default','Set to Random','Select Manually','Use Default');
    % Handle response
    switch answer
        case 'Use Default'
            disp('Using default initial conditions. NOTE: Currently defaulting to RANDOM')
        case 'Set to Random'
            disp('Setting initial conditions to random location on interval')
            initialCondition = cell(1,N);
            x = zeros(1,N); 
            y = zeros(1,N); 
            for i=1:N
                rand_width=FIELD_WIDTH*.9;
                x(i)=rand(1)*(2*rand_width)-rand_width;
                y(i)=rand(1)*(2*rand_width)-rand_width;
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
        case 'Select Manually'
            disp('Opening GUI interface for selection')
            fig=figure;

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
            initialCondition = cell(1,N);
            x = zeros(1,N); 
            y = zeros(1,N); 
            z = zeros(1,N);
            for i=1:N
                [x(i),y(i)] = ginput(1);
                z(i)=readScalarField(x(i),y(i));
                plot3(x(i),y(i),z(i)+abs(z(i)*.2),'o','MarkerSize',10,'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','k')
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
            close(fig);
    end
end


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



