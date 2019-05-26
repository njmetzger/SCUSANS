% <SIM_RUNNER> - Top-level parallel simulation caller.
%
% Configures specified block parameters of a given simulation sequence,
% runs the simulations in parallel, and processes the results. 
%
% Syntax:  sim_runner
%

clear all;

simulation_name = 'FindMax_Parallel';

% Batch simulation parameters 
for i=1:(20^2)
    times(i) = [60]*1; 
end
Nrobots= [8];
SIM_TIME= times(1);
Niterations = numel(times); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configure base robot template and project path
[project_path] = ConfigureProjectPath;
base = 'Swarm_Robot_Base_2018b';
open_system(base);
[base,cfg] = ConfigureRobotBehaviorTemplate(base);
cfg.simulation_name = [simulation_name,'_',num2str(Nrobots),'_',num2str(Niterations)];

% Create system with that number of robots
[h_newsys , simName] = BuildSimModel(Nrobots,base);
close_system(base,0);

% Configure swarm system parameters 
set_param('Swarm_Robot_N','Solver', 'FixedStepDiscrete');
set_param('Swarm_Robot_N','FixedStep', '0.1')
save_system('Swarm_Robot_N');

% Setup SimulationInput Object 
for i = Niterations:-1:1
    
    clear simIn_with_ICs
    
    simIn(i) = Simulink.SimulationInput('Swarm_Robot_N');
    simIn(i) = simIn(i).setModelParameter('StopTime',num2str(times(i)));
    
    [simIn(i)] = SetRandomRobotInitialConditions(simIn(i), Nrobots, cfg.field_width);
end

%%  Run parsim
out = parsim(simIn, 'ShowSimulationManager','on','ShowProgress','on','TransferBaseWorkspaceVariables','off');

%% Post - Processing
AnalyzeTimingData(out,times,Nrobots)
[data] = ExtractRobotData(out,cfg,Nrobots);
PlotCompositeRobotData(data(1))
PlotCompositeRobotData(data)
ProduceSimVideo(data)

% Save simulation results 
if exist([cd,'\results\',filesep,data(1).cfg.simulation_name,'.mat']) == 2
    data_filename=[cd,'\results\',data(1).cfg.simulation_name,'_',datestr(now,'yyyy-mm-dd'),'_',datestr(now,'HH_MM_SS'),'.mat'];
else
    data_filename=[cd,'\results\',data(1).cfg.simulation_name,'.mat'];
end
save(data_filename,'data');


%% Subfunctions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% Sub - Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

%% AnalyzeTimingData()
function AnalyzeTimingData(out,times,Nrobots)

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
fprintf('Real-Time Factor: %0.2f s\n',SIM_TIME/(sum(runtimes)/length(runtimes)) )
disp('-------------------------------------')
min_t = min(runtimes);
max_t = max(runtimes);
avg_t = sum(runtimes)/length(runtimes);
realtime_f = SIM_TIME/(sum(runtimes)/length(runtimes));

for j = 1:length(Nrobots)
    for i = 1:length(times)
        min_tm(i,j) = min_t;
        max_tm(i,j) = max_t;
        avg_tm(i,j) = avg_t;
        rt_fm(i,j) = realtime_f;
        discript(i,j) = sprintf("NRobots=%d__Time=%d",Nrobots(j),times(i));
%         save('attractR200_T100_test','min_tm','max_tm','avg_tm','rt_fm','discript')
    end
end
end

%% ConfigureRobotBehaviorTemplate()
function [base,cfg] = ConfigureRobotBehaviorTemplate(base)

% Configuration Parameters
cfg.SENSOR_RANGE= 300;
cfg.AVOID_RANGE= 60;
cfg.DESIRED_VALUE= 1;
cfg.CONTOUR_BUFFER= 1;

cfg.x_init_center= 100;
cfg.y_init_center= 100;
cfg.init_radius= 1;
cfg.GoTo_Coords = [100, 100];

cfg.behavior = 'Find Max';
cfg.ScalarFieldSelection = 1;

% Set template behavior switches
set_param(strcat(base,'/Robot 1 Behavior/Attract_Switch'),'sw','1')
set_param(strcat(base,'/Robot 1 Behavior/Disperse_Switch'),'sw','0')
set_param(strcat(base,'/Robot 1 Behavior/FindMin_Switch'),'sw','0')
set_param(strcat(base,'/Robot 1 Behavior/FindMax_Switch'),'sw','1')
set_param(strcat(base,'/Robot 1 Behavior/FollowContour_Switch'),'sw','0')
set_param(strcat(base,'/Robot 1 Behavior/FollowRidge_Switch'),'sw','0')
set_param(strcat(base,'/Robot 1 Behavior/FollowTrench_Switch'),'sw','0')
set_param(strcat(base,'/Robot 1 Behavior/GoTo_Switch'),'sw','0')

set_param(strcat(base,'/Robot 1 Behavior/Robot Speed'),'value', '1.0');   %0.5
set_param(strcat(base,'/Robot 1 Behavior/GoTo_X'),'value','50');
set_param(strcat(base,'/Robot 1 Behavior/GoTo_Y'),'value','100');

set_param(strcat(base,'/Robot 1 Behavior/Sensor Range'),'Value',num2str(cfg.SENSOR_RANGE));
set_param(strcat(base,'/Robot 1 Behavior/Avoid Range'),'Value',num2str(cfg.AVOID_RANGE));
set_param(strcat(base,'/Robot 1 Behavior/Desired Value'),'Value',num2str(cfg.DESIRED_VALUE));
set_param(strcat(base,'/Robot 1 Behavior/Contour Buffer'), 'Value',num2str(cfg.CONTOUR_BUFFER));
set_param(strcat(base,'/Robot 1 SimResponse/ScalarFieldSelection'),'Value',num2str(cfg.ScalarFieldSelection));

[~,cfg.field_width] = readScalarField(0,0,cfg.ScalarFieldSelection);
end


%% SetRandomRobotInitialConditions()
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


%% ConfigureProjectPath()
function [project_path] = ConfigureProjectPath
[project_path] = fileparts(mfilename('fullpath'));
cd(project_path);
addpath(fullfile(project_path,'utilities'),fullfile(project_path,'utilities/testbed'),fullfile(project_path,'utilities/behaviors'));
addpath(fullfile(project_path,'results'),fullfile(project_path,'results/videos'));
end

