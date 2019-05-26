function [h_newsys , simName] = BuildSimModel(N,base)
% <BUILDSIMMODEL> - Constructs simulink model for N robots given a standard
% robot template. 
%
% Syntax:  [h_newsys , simName] = BuildSimModel(N,base)
%
% Inputs:
%    N - Number of robots in simulation
%    base - b
%
% Outputs:
%    h_newsys - handle to swarm simulation model.  
%    simName - name of swarm simulation model. 
%

% Define system name
simName='Swarm_Robot_N';

% Default vertical spacing of blocks
vert_spacing = -150;

% If a system is already open with the same name, close it without saving
if exist('Swarm_Robot_N')
    close_system('Swarm_Robot_N',0);
end

% Create new blank simulink model
h_newsys = new_system('Swarm_Robot_N');
open_system(h_newsys);

for i=1:N
    %% Create new "Robot X Behavior" block
    base_model=strcat(base,'/Robot 1 Behavior');
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i);
    addBlockAndSpace(base_model, behavior_model,vert_spacing, 0, i );

    h_behavior{i} = get_param(behavior_model,'PortHandles');

    base_model=strcat(base,'/Robot 1 SimResponse');
    response_model=sprintf('Swarm_Robot_N/Robot %i SimResponse',i);
    addBlockAndSpace(base_model, response_model,vert_spacing, 0, i );
    
    b_behavior{i} = get_param(response_model,'PortHandles');
    pos=get_param(response_model,'position');

    %% Create new "Robot X" (robot num) source block
    base_model=strcat(base,'/Robot 1');
    new_model=sprintf('Swarm_Robot_N/Robot %i',i);
    addBlockAndSpace(base_model, new_model,vert_spacing, 0, i );

    set_param(new_model,'Value',num2str(i));

    h_robot_num{i} = get_param(new_model,'PortHandles');

    % Connect robot num to behavior block
    add_line('Swarm_Robot_N',h_robot_num{i}.Outport(1),h_behavior{i}.Inport(2));

    %% Connect communication blocks 
    % Connect robot behavior block to robot response block
    add_line('Swarm_Robot_N', h_behavior{i}.Outport(1), b_behavior{i}.Inport(1));

end

% Add Mux block for all robot signals
add_block('simulink/Signal Routing/Mux','Swarm_Robot_N/Mux');

set_param('Swarm_Robot_N/Mux','Inputs',num2str(N));
set_param('Swarm_Robot_N/Mux','Orientation','down');

%[left top right bottom]
last_pos=pos;
pos=last_pos;
pos(1)=-1167 ;
pos(2)=last_pos(2)-vert_spacing;
pos(3)=-969;
pos(4)=pos(2)+15;
set_param('Swarm_Robot_N/Mux','position',pos);

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

end

%% addBlockAndSpace()
function [] = addBlockAndSpace(base_model, new_model,vert_spacing, hor_spacing, i )
    add_block(base_model, new_model);
    pos=get_param(new_model,'position');
    pos(1)=pos(1)-hor_spacing; 
    pos(3)=pos(3)-hor_spacing;
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    set_param(new_model,'position',pos);
end