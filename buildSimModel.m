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
            fig=figure;
            ax = axes();
            cmap = hsv(N);
            title('Click to select robot initial positions')
            
            res=100;
            [s,Z] = PlotScalarField(ax,ScalarFieldSelection,res,SimParams.DESIRED_VALUE,FIELD_WIDTH,'');
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

%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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