function Swarm_Robot_Test_Sim()

% Define number of robots 
NUM_ROBOTS=10; 

% Create system with that number of robots 
[h_newsys , simName] = buildSimModel(NUM_ROBOTS)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Group Robot Behavior Control

% Prompt user to select desired behavior 
cmdList = {'Gather','Disperse','Home','Wander','Follow','Velocity','Flock','Disperse&Wander'};

[indx,tf] = listdlg('PromptString','Select a command:',...
    'SelectionMode','single','ListString',cmdList);

% Check if inputs are valid 
if isempty(indx)
    msgbox('Selection Cancelled.');
    return;
end

switch cmdList{indx}
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Gather'
        for i=1:NUM_ROBOTS
            set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '1');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Disperse'
        for i=1:NUM_ROBOTS
            set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '2');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Home'
        prompt = {'Enter X position', 'Enter Y position'};
        p = inputdlg(prompt, 'Input Desired Position');
        
        x_pos = (p{1});
        y_pos = (p{2});
        
        for i=1:NUM_ROBOTS
            set_param('Swarm_Robot_N/Command','Value', '3');
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Homing Behavior/x_pos'], 'Value', x_pos);
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Homing Behavior/y_pos'], 'Value', y_pos);
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Wander'
        for i=1:NUM_ROBOTS
            set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '4');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Follow'
        for i=1:NUM_ROBOTS
            set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '5');
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Velocity'
        
        vel_choice = questdlg('Specify Velocity Type','Velocity Type',...
            'Cardinal Directions (N,S,E,W)','Specific Velocity (x,y)',...
            'Cancel','Cancel');
        
        doVelocityBehavior(vel_choice, NUM_ROBOTS)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 'Flock'
        %A combination of Safe wander, gather, and homing
        validInput = 1;
        
        for i=1:NUM_ROBOTS
            set_param(['Swarm_Robot_N/Command ',num2str(i)],'Value', '7');
        end
        
        %Parameters for Homing
        prompt={'Enter X Position', 'Enter Y Position'};
        d = inputdlg(prompt, 'Input Desired Positions');
        if isempty(d)
            msgbox('Selection cancelled.');
            return
        else
            x_pos = d{1};
            y_pos = d{2};
        end
        
        for i=1:NUM_ROBOTS     
            set_param(['Swarm_Robot_N/Robot ', num2str(i),' Behavior/Homing Behavior/x_pos'],'Value', x_pos);
            set_param(['Swarm_Robot_N/Robot ', num2str(i),' Behavior/Homing Behavior/y_pos'],'Value', y_pos);
        end
        
    case 'Disperse&Wander'
        set_param('Swarm_Robot_N/Command','Value', '8');
        
    otherwise
        msgbox('Try again with a valid input.')
end
    
%Runs Simulation, must be the name of the simulink file without the .slx
%extention

% profile on

simOut=sim('Swarm_Robot_N','StopTime', '3');
% simOut=sim('Swarm_Robot_N');

Robot1=simOut.Robot1;
Robot2=simOut.Robot2;
Robot3=simOut.Robot3;
Robot4=simOut.Robot4;
Robot5=simOut.Robot5;
Robot6=simOut.Robot6;
Robot7=simOut.Robot7;
Robot8=simOut.Robot8;
Robot9=simOut.Robot9;
Robot10=simOut.Robot10;


%% Plots robots positions

cmap = hsv(length(Robot1.data));

figure(1)

% First plot scalar field 
ax=gca;
ax.XLim=[-5 5];
ax.YLim=[-5 5];

res=100;
xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
[X,Y] = meshgrid(xdivs,ydivs);
Z=readScalarField2(X,Y);
%Z= @(x,y) 3*(1-x).^2.*exp(-(x.^2) - (y+1).^2) ... 
%    - 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2) ... 
%    - 1/3*exp(-(x+1).^2 - y.^2);

surf(X,Y,Z);
view([0 90])

figure(1)
axis([-5 5 -5 5])

contour(Z)
view([0 90])
axis([-5 5 -5 5])

for i=1:length(Robot1.data)
    hold on
    
    
    
    plot(Robot1.data(i,1),Robot1.data(i,2),'ob','MarkerSize',5,'MarkerFaceColor','b')
    plot(Robot2.data(i,1),Robot2.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','g')
    plot(Robot3.data(i,1),Robot3.data(i,2),'or','MarkerSize',5,'MarkerFaceColor','r')
    plot(Robot4.data(i,1),Robot4.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','c')
    plot(Robot5.data(i,1),Robot5.data(i,2),'om','MarkerSize',5,'MarkerFaceColor','m')
    plot(Robot6.data(i,1),Robot6.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','y')
    plot(Robot7.data(i,1),Robot7.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','k')
    plot(Robot8.data(i,1),Robot8.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','w')
    plot(Robot9.data(i,1),Robot9.data(i,2),'sb','MarkerSize',5,'MarkerFaceColor','b')
    plot(Robot10.data(i,1),Robot10.data(i,2),'sk','MarkerSize',5,'MarkerFaceColor','g')
    axis([-5 5 -5 5])
    
    hold off
    
    drawnow limitrate 
    clf
end

%Shows final Position
hold on
plot(Robot1.data(i,1),Robot1.data(i,2),'ob','MarkerSize',5,'MarkerFaceColor','b')
plot(Robot2.data(i,1),Robot2.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','g')
plot(Robot3.data(i,1),Robot3.data(i,2),'or','MarkerSize',5,'MarkerFaceColor','r')
plot(Robot4.data(i,1),Robot4.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','c')
plot(Robot5.data(i,1),Robot5.data(i,2),'om','MarkerSize',5,'MarkerFaceColor','m')
plot(Robot6.data(i,1),Robot6.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','y')
plot(Robot7.data(i,1),Robot7.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','k')
plot(Robot8.data(i,1),Robot8.data(i,2),'ok','MarkerSize',5,'MarkerFaceColor','w')
plot(Robot9.data(i,1),Robot9.data(i,2),'sb','MarkerSize',5,'MarkerFaceColor','b')
plot(Robot10.data(i,1),Robot10.data(i,2),'sk','MarkerSize',5,'MarkerFaceColor','g')
%plot(x11.data(i),y11.data(i),'sk','MarkerSize',5,'MarkerFaceColor','r')
axis([-5 5 -5 5]);
surf(X,Y,Z-20)
view([0 90])
hold off

% profile viewer

end


%%
%%%%%%%%%%%%%%%%%%%%%%%%%% SUB FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% doVelocityBehavior()
function doVelocityBehavior(vel_choice, NUM_ROBOTS)


switch vel_choice
    case 'Cardinal Directions (N,S,E,W)'
        
        list = {'North','South','East','West'};
        [indx,tf] = listdlg('ListString',list,'SelectionMode','single');
        
        switch list{indx}
            case 'North'
                x_vel = '0';
                y_vel = '1';
            case 'South'
                x_vel = '0';
                y_vel = '-1';
            case 'East'
                x_vel = '1';
                y_vel = '0';
            case 'West'
                x_vel = '-1';
                y_vel = '0';
            otherwise   %Cancelled
                vel_switch='Cancel';
                x_vel = '0';
                y_vel = '0';
                fprintf(2,'Velocity specification cancelled');
        end
        
    case 'Specific Velocity (x,y)'
        % Prompt user to input desired X and Y Velocities
        prompt={'Enter X velocity', 'Enter Y Velocity'};
        d = inputdlg(prompt, 'Input Desired Velocity');
        
        if ~isempty(d)
            x_vel = d{1};
            y_vel = d{2};
        end
        
    otherwise
        fprintf(2,'Invalid velocity behavior input.')
end
        

% Set simulation parameters for each robot
for i=1:NUM_ROBOTS
    set_param(['Swarm_Robot_N/Command'],'Value', '6');
    set_param(['Swarm_Robot_N/Robot ', num2str(i),' Behavior/Generalized Direction Command/X_vel'],'Value', x_vel);
    set_param(['Swarm_Robot_N/Robot ', num2str(i),' Behavior/Generalized Direction Command/Y_vel'],'Value', y_vel);
end

end

%% buildSimModel()
function [h_newsys , simName] = buildSimModel(N)
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

% Construct total system of N robots and link blocks 
for i=1:N
    
    %% Create new "Robot X Behavior" block
    base_model='Swarm_Robot_Base/Robot 1 Behavior';
    behavior_model=sprintf('Swarm_Robot_N/Robot %i Behavior',i)

    add_block(base_model,behavior_model);
    
    pos=get_param(behavior_model,'position');
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    
    set_param(behavior_model,'position',pos);
    
    h_behavior{i} = get_param(behavior_model,'PortHandles');
    
    
    %% Create new "Robot X" (robot num) source block
    base_model='Swarm_Robot_Base/Robot 1';
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
    base_model='Swarm_Robot_Base/Command 1';
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
   
    %% Create new "RobotX" (robot data) sink block
    base_model='Swarm_Robot_Base/Robot1_Data';
    new_model=['Swarm_Robot_N/Robot',num2str(i),'_Data'];
    
    add_block(base_model,new_model);
    
    pos=get_param(new_model,'position');
    pos(2)=pos(2)-vert_spacing*(i-1);
    pos(4)=pos(4)-vert_spacing*(i-1);
    
    set_param(new_model,'position',pos);
    set_param(new_model,'VariableName',['Robot',num2str(i)]);
    
    h_outdata{i} = get_param(new_model,'PortHandles');
    
    % Connect robot num to behavior block
   add_line('Swarm_Robot_N',h_behavior{i}.Outport(1),h_outdata{i}.Inport(1));

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
            x(i)=rand(1)*10-5;
            y(i)=rand(1)*10-5;
            initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Initial Conditions'],'Value',initialCondition{i})
        end
    case 'Select Manually'
        disp('Opening GUI interface for selection')
        fig=figure
        
        % First plot scalar field
        ax=gca;
        ax.XLim=[-5 5];
        ax.YLim=[-5 5];
        cmap = hsv(N);
        title('Click to select robot initial positions')
        
        res=100;
        xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
        ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
        [X,Y] = meshgrid(xdivs,ydivs);
        Z=readScalarField2(X,Y);
        surf(X,Y,Z);
        view([0 90])
        hold on
        
        for i=1:N
            [x(i),y(i)] = ginput(1)
            z(i)=readScalarField2(x(i),y(i));
            plot3(x(i),y(i),z(i)+abs(z(i)*.2),'o','MarkerSize',10,'MarkerFaceColor',cmap(i,:),'MarkerEdgeColor','k')
            initialCondition{i}=sprintf('[%g %g 0]',x(i),y(i));
            set_param(['Swarm_Robot_N/Robot ',num2str(i),' Behavior/Initial Conditions'],'Value',initialCondition{i})
        end
        close(fig);
end


end





