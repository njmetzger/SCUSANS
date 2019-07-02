
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
    %Parallel sim trial
    simulation_name = strcat(behavior,'_Parallel');
    for r = 1:length(SimParams.NUM_ROBOTS)
        Nrobots = SimParams.NUM_ROBOTS(r);
        SimParamsi = SimParams;
        SimParamsi.NUM_ROBOTS = Nrobots;
        clear simIn_with_ICs
        [h_newsys , simName] = buildSimModel(SimParamsi,FIELD_WIDTH, ScalarFieldSelection,trialType,base);
        close_system(base,0);
        set_param('Swarm_Robot_N','Solver', 'FixedStepDiscrete');
        set_param('Swarm_Robot_N','FixedStep', '0.1')
        save_system('Swarm_Robot_N');
        for t = 1:length(SimParams.SIM_TIME)
            time = SimParams.SIM_TIME(t);
            SimParamsi.SIM_TIME = time;
            cfg.simulation_name = [simulation_name,'_R',num2str(Nrobots),'_T',num2str(time),'_N',num2str(SimParams.NumTrials)];
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
            [data] = ExtractRobotData(out,cfg,Nrobots);
            PlotCompositeRobotData(data(1),ScalarFieldSelection,behavior,FIELD_WIDTH)
            %PlotCompositeRobotData(data,ScalarFieldSelection,behavior,FIELD_WIDTH)
            if SimParams.MakeVideo
                ProduceSimVideo(data,ScalarFieldSelection,behavior,FIELD_WIDTH,SimParams)
            end
            % Save simulation results
            %if exist(fullfile(cd,'results',filesep,cfg.simulation_name,'.mat')) == 2
            %    data_filename= fullfile(cd,'results',cfg.simulation_name,'_',[datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS')],'.mat');
            %else
            %    data_filename=fullfile(cd,'results',[cfg.simulation_name,'.mat']);
            %end
            %save(data_filename,'data');
        end
        delete Swarm_Robot_N.slx
    end
    output_stats = fullfile(cd,'results',[behavior,'-R_',num2str(min(SimParams.NUM_ROBOTS)),'-',num2str(max(SimParams.NUM_ROBOTS)),'-T_',num2str(min(SimParams.SIM_TIME)),'-',num2str(max(SimParams.SIM_TIME)),'-N_',num2str(SimParams.NumTrials),'.mat']);
    save(output_stats,'avg_tm','discript','max_tm','min_tm','rt_fm')

else
    % single simulation or experimental trial
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
    delete Swarm_Robot_N.slx
end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% ExtractRobotData returns an array of structs each with 4 parts
% 1) out : an array of all associated simouts
% 2) cfg : which is a struct of configuration parameters
% 3) robot: which is an array of all the data for each robot. This should
% be used for post processing
% 4) robot_DS: which is a resampled version of robot, this data is 
% resampled to 2 Hz for convenient plotting, robot shoudl be used for
% plotting only
% NUM_SIGNALS_PER_ROBOT=4;
% est_num_robots=size(simOut.simout.Data,2)/NUM_SIGNALS_PER_ROBOT;

for i=1:numel(out)
    data(i).out = out(i);
    data(i).cfg = cfg;
    data(i).cfg.Nrobots = Nrobots;
    
    desiredFPS=2;
    maxtime= max(out(i).simout.Time);
    desiredNumFrames=desiredFPS*maxtime;
    uniform_time=linspace(0,maxtime,desiredNumFrames);
    resamp_data=resample(out(i).simout,uniform_time);
    
    % Extract individual robot data and organize into structure array
    for j=1:Nrobots
        %Build original robot array
        data(i).robot(j).x=out(i).simout.Data(:,j*4-3);
        data(i).robot(j).y=out(i).simout.Data(:,j*4-2);
        data(i).robot(j).theta=out(i).simout.Data(:,j*4-1);
        data(i).robot(j).sensor_value=out(i).simout.Data(:,j*4);
        data(i).robot(j).time=out(i).simout.time;
        
        %Build downsampled robot_ds array
        data(i).robot_ds(j).x=resamp_data.Data(:,j*4-3);
        data(i).robot_ds(j).y=resamp_data.Data(:,j*4-2);
        data(i).robot_ds(j).theta=resamp_data.Data(:,j*4-1);
        data(i).robot_ds(j).sensor_value=resamp_data.Data(:,j*4);
        data(i).robot_ds(j).time=resamp_data.time;
    end
end
end

