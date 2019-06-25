% function Swarm_Robot_Test_Sim(NUM_ROBOTS,Sim_Time,SensorRange,AvoidRange,...
%     DesiredValue,CONTOUR_BUFFER,GoTo,ScalarFieldSelection,behavior,x_init,y_init,...
%     radius_init,trialType, robots,base)
function Swarm_Robot_Test_Sim(SimParams,ScalarFieldSelection,behavior,trialType,base)
% SWARM_ROBOT_TEST_SIM - < Setup and initialization utility for running the
% swarm simulator.>


% Constants
NUM_SIGNALS_PER_ROBOT=4;
if ScalarFieldSelection == 4
    FIELD_WIDTH=5;
elseif ScalarFieldSelection == 5
    FIELD_WIDTH=1500;
else
    FIELD_WIDTH=300;
end


% Setup matlab directory to run in current runFile folder
runFileFunctionName=mfilename;
runFilePath=mfilename('fullpath');
BaseFolderPath = replace(runFilePath,runFileFunctionName,string(''));
cd(BaseFolderPath);
[~] = ConfigureProjectPath;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Group Robot Behavior Control

%Runs Simulation, must be the name of the simulink file without the .slx
%extention

% profile on

% Specify sim options and run simulation

if trialType ==2
    simulation_name = strcat(behavior,'_Parallel');
    for t = 1:length(SimParams.SIM_TIME)
        time = SimParams.SIM_TIME(t);
        for r = 1:length(SimParams.NUM_ROBOTS)
            Nrobots = SimParams.NUM_ROBOTS(r);
            SimParamsi = SimParams;
            SimParamsi.NUM_ROBOTS = Nrobots;
            SimParamsi.SIM_TIME = time;
            clear simIn_with_ICs
            cfg.simulation_name = [simulation_name,'_R',num2str(Nrobots),'_T',num2str(time),'_N',num2str(SimParams.NumTrials)];
            
            [h_newsys , simName] = buildSimModel(SimParamsi,FIELD_WIDTH, ScalarFieldSelection,trialType,base);
            close_system(base,0);
            set_param('Swarm_Robot_N','Solver', 'FixedStepDiscrete');
            set_param('Swarm_Robot_N','FixedStep', '0.1')
            save_system('Swarm_Robot_N');
            for i = SimParams.NumTrials:-1:1
                simIn(i) = Simulink.SimulationInput('Swarm_Robot_N');
                simIn(i) = simIn(i).setModelParameter('StopTime',num2str(time));
                
                [simIn(i)] = SetRandomRobotInitialConditions(simIn(i), Nrobots, FIELD_WIDTH);
            end
            out = parsim(simIn, 'ShowSimulationManager','on','ShowProgress','on','TransferBaseWorkspaceVariables','off');
            %% Post - Processing
            [min_t, max_t,avg_t,realtime_f] =AnalyzeTimingData(out,time,Nrobots);
            min_tm(t,r) = min_t;
            max_tm(t,r) = max_t;
            avg_tm(t,r) = avg_t;
            rt_fm(t,r) = realtime_f;
            discript(t,r) = sprintf("NRobots=%d_Time=%d_N=%d",Nrobots,time,SimParams.NumTrials);
            % save('attractR200_T100_test','min_tm','max_tm','avg_tm','rt_fm','discript')
            [data] = ExtractRobotData(out,cfg,Nrobots);
            PlotCompositeRobotData(data(1),ScalarFieldSelection,behavior,FIELD_WIDTH)
            PlotCompositeRobotData(data,ScalarFieldSelection,behavior,FIELD_WIDTH)
            if SimParams.MakeVideo
                ProduceSimVideo(data,ScalarFieldSelection,behavior,FIELD_WIDTH,SimParams)
            end
            delete Swarm_Robot_N.slx
            % Save simulation results
            %if exist(fullfile(cd,'results',filesep,cfg.simulation_name,'.mat')) == 2
            %    data_filename= fullfile(cd,'results',cfg.simulation_name,'_',[datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS')],'.mat');
            %else
            %    data_filename=fullfile(cd,'results',[cfg.simulation_name,'.mat']);
            %end
            %save(data_filename,'data');
        end
    end
    output_stats = fullfile(cd,'results',[behavior,':','R_',num2str(min(SimParams.NUM_ROBOTS)),'-',num2str(max(SimParams.NUM_ROBOTS)),':T_',num2str(min(SimParams.SIM_TIME)),'-',num2str(max(SimParams.SIM_TIME)),':N_',num2str(SimParams.NumTrials),'.mat']);
    save(output_stats,'avg_tm','discript','max_tm','min_tm','rt_fm')
else
    % Create system with that number of robots
    cfg.simulation_name = [behavior,'_R',num2str(SimParams.NUM_ROBOTS),'_T',num2str(SimParams.SIM_TIME)];
    [h_newsys , simName] = buildSimModel(SimParams,FIELD_WIDTH, ScalarFieldSelection,trialType,base);
    set_param('Swarm_Robot_N','Solver', 'FixedStepDiscrete');
    set_param('Swarm_Robot_N','FixedStep', '0.1')
    save_system('Swarm_Robot_N');
    sim('Swarm_Robot_N',SimParams.SIM_TIME);
    SimOut.simout = simout;
    [data] = ExtractRobotData(SimOut,cfg,SimParams.NUM_ROBOTS);
    if SimParams.MakeVideo
        ProduceSimVideo(data,ScalarFieldSelection,behavior,FIELD_WIDTH,SimParams)
    end
    
    %% Plot time history of robots
    plotRobotHistory(data,SimParams,behavior,ScalarFieldSelection,FIELD_WIDTH);
end
end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% buildSimModel()
function [h_newsys , simName]= buildSimModel(SimParams,FIELD_WIDTH, ScalarFieldSelection,trialType,base)
% Script for generating simulink model for N robots

% Define system name
simName='Swarm_Robot_N';
N = SimParams.NUM_ROBOTS;

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
open_system(base)

% Set base robot parameters
set_param(strcat(base,'/Robot 1 Behavior/Sensor Range'),'Value',num2str(SimParams.SENSOR_RANGE));
set_param(strcat(base,'/Robot 1 Behavior/Avoid Range'),'Value',num2str(SimParams.AVOID_RANGE));
set_param(strcat(base,'/Robot 1 Behavior/Desired Value'),'Value',num2str(SimParams.DESIRED_VALUE));
set_param(strcat(base,'/Robot 1 Behavior/Contour Buffer'), 'Value',num2str(SimParams.CONTOUR_BUFFER));
set_param(strcat(base,'/Robot 1 SimResponse/ScalarFieldSelection'),'Value',num2str(ScalarFieldSelection));

% Construct total system of N robots and link blocks
if trialType == 3
    rs = string('{');
    for j = 1:length(SimParams.robots)
        if j ==1
            rs = rs+ sprintf(string('''%s'''),SimParams.robots(j));
        else
            rs = rs+ sprintf(',''%s''', SimParams.robots(j));
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
    if trialType == 3
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
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i);
    addBlockAndSpace(base_model, behavior_model,vert_spacing, 0, i )
    
    h_behavior{i} = get_param(behavior_model,'PortHandles');
    
    %% Create new "Robot X Response" block
    if trialType == 3
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
    new_model=sprintf('Swarm_Robot_N/Robot %i',i);
    addBlockAndSpace(base_model, new_model,vert_spacing, 0, i )
    
    set_param(new_model,'Value',num2str(i));
    
    h_robot_num{i} = get_param(new_model,'PortHandles');
    
    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_robot_num{i}.Outport(1),h_behavior{i}.Inport(2));
    
    
    %% Connect communication blocks
    % Connect robot behavior block to robot response block
    add_line('Swarm_Robot_N', h_behavior{i}.Outport(1), b_behavior{i}.Inport(1));
    
    %Add communication blocks for experiment robots
    if trialType == 3
        new_model = sprintf('Swarm_Robot_N/Robot %d ExpResponse/Robot_command', i);
        set_param(new_model, 'robotName', SimParams.robots(i))
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

%% Define robot initial conditions
if trialType == 1
    answer = questdlg('Select initial condition option?', ...
        'Initial Condition Options', ...
        'Use Default','Set to Random','Select Manually','Use Default');
    % Handle response
    switch answer
        case 'Use Default'
            disp('Using default initial conditions.')
            % set initial conditions as center point and circle:
            center_point_x= SimParams.init.x_init_center;
            center_point_y= SimParams.init.y_init_center;
            radius_circle= SimParams.init.init_radius;
            theta_offset= 2*pi/N;
            for i= 1:N
                x(i) = center_point_x + radius_circle*cos(i*theta_offset);
                y(i)= center_point_y + radius_circle*sin(i*theta_offset);
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
            
        case 'Set to Random'
            disp('Setting initial conditions to random location on interval')
            
            for i=1:N
                rand_width=FIELD_WIDTH*.9;
                x(i)=rand(1)*(2*rand_width)-rand_width;
                y(i)=rand(1)*(2*rand_width)-rand_width;
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
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
            if ScalarFieldSelection ~=5
                Z=readScalarField(X,Y,ScalarFieldSelection);
            else
                for i = 1:length(X)
                    for j = 1:length(Y)
                        Z(i,j)=readScalarField(X(i,j),Y(i,j),ScalarFieldSelection);
                    end
                end
            end
            surf(X,Y,Z);
            view([0 90])
            hold on
            
            for i=1:SimParams.NUM_ROBOTS
                [x(i),y(i)] = ginput(1);
                z(i)=readScalarField(x(i),y(i),ScalarFieldSelection);
                plot3(x(i),y(i),z(i)+abs(z(i)*.2),'o','MarkerSize',10,'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','k')
                initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
                set_param(['Swarm_Robot_N/Robot ',num2str(i),' SimResponse/Initial Conditions'],'Value',initialCondition{i})
            end
            close(fig);
    end
end


end



%% addBlockAndSpace()
function [] = addBlockAndSpace(base_model, new_model,vert_spacing, hor_spacing, i )
add_block(base_model, new_model)
pos=get_param(new_model,'position');
pos(1)=pos(1)-hor_spacing;
pos(3)=pos(3)-hor_spacing;
pos(2)=pos(2)-vert_spacing*(i-1);
pos(4)=pos(4)-vert_spacing*(i-1);
set_param(new_model,'position',pos);
end
%% ConfigureProjectPath()
function [project_path] = ConfigureProjectPath
[project_path] = fileparts(mfilename('fullpath'));
cd(project_path);
if exist(fullfile(project_path,'results'))==0
    mkdir(project_path,'results')
    mkdir(fullfile(project_path,'results'),'videos')
end

addpath(fullfile(project_path,'utilities'),fullfile(project_path,'utilities/testbed'),fullfile(project_path,'utilities/behaviors'));
addpath(fullfile(project_path,'results'),fullfile(project_path,'results/videos'));
end

%% SetRandomRobotInitialConditions
function [simIn_with_ICs] = SetRandomRobotInitialConditions(simIn, Nrobots, field_width)

rand_width=field_width*.6;

for j=1:Nrobots
    x(j)=rand(1)*(2*rand_width)-rand_width;
    y(j)=rand(1)*(2*rand_width)-rand_width;
    initialCondition{j}=sprintf('[%g %g 0]',x(j),y(j));
    simIn = setBlockParameter(simIn,['Swarm_Robot_N/Robot ',num2str(j),' SimResponse/Initial Conditions'],'Value',initialCondition{j});
end

simIn_with_ICs = simIn;
end

%% AnalyzeTimingData()
function [min_t, max_t,avg_t,realtime_f]= AnalyzeTimingData(out,times,Nrobots)

SIM_TIME = times(1);

disp('parallel results processing here')
for i = 1:length(out)
    runtimes(i) = out(i).SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
end

disp('-------Simulation Analysis-----------')
fprintf('Simulation Runtime: %0.2f s\n',SIM_TIME)
fprintf('Min Wall Runtime: %0.2f s\n',min(runtimes))
fprintf('Max Wall Runtime: %0.2f s\n',max(runtimes))
fprintf('Average Wall Runtime: %0.2f s\n', sum(runtimes)/length(runtimes))
fprintf('Real-Time Factor: %0.2f \n',SIM_TIME/(sum(runtimes)/length(runtimes)) )
disp('-------------------------------------')
min_t = min(runtimes);
max_t = max(runtimes);
avg_t = sum(runtimes)/length(runtimes);
realtime_f = SIM_TIME/(sum(runtimes)/length(runtimes));

end

%% ExtractRobotData()
function [data] = ExtractRobotData(out,cfg,Nrobots)
% NUM_SIGNALS_PER_ROBOT=4;
% est_num_robots=size(simOut.simout.Data,2)/NUM_SIGNALS_PER_ROBOT;

for i=1:numel(out)
    data(i).out = out(i);
    data(i).cfg = cfg;
    data(i).cfg.Nrobots = Nrobots;
    
    % Extract individual robot data and organize into structure array
    for j=1:Nrobots
        data(i).robot(j).x=out(i).simout.Data(:,j*4-3);
        data(i).robot(j).y=out(i).simout.Data(:,j*4-2);
        data(i).robot(j).theta=out(i).simout.Data(:,j*4-1);
        data(i).robot(j).sensor_value=out(i).simout.Data(:,j*4);
        data(i).robot(j).time=out(i).simout.time;
    end
end
end

