function [Vf,SwarmStateNum] = SwarmSimRidgeFollow_test(RobotParams, NRobot, SensorRange, ScalarFieldSelection)
%SwarmSimRidgeFollow_test New attempt at ridge following algorithm

% NOTE: currently assuming that all robots will be in range - can add a
% switch later that changes this
%   SSRF_test assigns robot "roles" on the fly based on the comparison of
%   local values. Roles are as follows:
%           "Master" - the robot with the higher reading along the
%           shallowest gradient
%           "Leader" - the robot with the lower reading along the
%           shallowest gradient
%           "Left" - robot to the left of the shallowest gradient line,
%           from the perspective of the "master" robot. The gradient from
%           Master to Left must be greater than than the gradient from
%           master to leader.
%           "Right" - robot to right of shallowest gradient line, same
%           perspective as for left robot.
%  After defining the roles, there are four possible cases:
%    1) All roles are populated AND master has the highest scalar value
%    (both booleans are true)
%           interpretation: the swarm is situtated on a ridge
%           output: all robots move along the bearing that defines the line
%          from master to leader
%    2) All roles are populated BUT master is not highest scalar value (one
%    true and one false)
%           interpretation: the swarm is not on a ridge
%           output: enter a "find max" behavior and continue to calculate
%           and monitor for the ridge roles
%    3) Not all roles are populated AND the master is the highest value
%    (one true and one false)
%           interpretation: the swarm may be on a ridge but is offset to
%           one side of it
%           output: move 90 degrees offset to the master/leader line in the
%           non-populated direction
%    4) not all roles are populated AND the master is not the highest value
%           interpretation: not on the ridge, and offset in one direction
%           output: enter a "find max" behavior

%% Initialize Variables
% Determine number of robots based off length of robot params vector
N= floor(length(RobotParams)/4);
x=zeros(1,N);
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N);
d=zeros(N,N);
O=zeros(N,N);
local_grad= zeros(N,N);
Vfx= 0;
Vfy= 0;
Vft= 0;
SwarmStateNum=0;
Vf= [Vfx Vfy Vft];

%%  Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Determine "building blocks" - distance, angle, and "gradient" to each robot
% Determine distance and angle to each robot
% different from other behaviors - NxN matrix rather than 1xN vector
for i=1:N
    for j= 1:N
        if i==j
            d(i,j)= NaN;
            O(i,j)= NaN;
        else
            d(i,j) = sqrt(((x(i)-x(j))^2 +(y(i)-y(j))^2));
            O(i,j) = atan2((y(j)-y(i)), (x(j)-x(i))); %do we need to mod this somehow?
        end
    end
end

% determine "local gradients" - delta sensor values divided by distance
% between the robots

for i=1:N
    for j= 1:N
        if i== j
            local_grad(i,j)= NaN;
        else
            local_grad(i)= (SensorValue(i)-SensorValue(j))/(d(i,j)) ;
        end
    end
end

%% Use the "building blocks" to determine robot "roles"

% Determine which robots lie on the lowest gradient line
[min_grad_row, min_grad_col]= find(abs(local_grad) == min(min(abs(local_grad))));

% add catch for if there are multiple mins in the local_grad
size_mgr= size(min_grad_row);
size_mgc= size(min_grad_col);

if size_mgr(1)>0
    min_grad_row=min_grad_row(1);
else
    min_grad_row= 1; %assume that the first robot is master if above fails
end

if size_mgc(1)>0
    min_grad_col=min_grad_col(1);
else
    min_grad_col= 2; %assume that the second robot is the leader if above fails
end

% determine which robot has the higher value and assign "Master" and
% "Leader" values accordingly
% master_robot_index=1;
% leader_robot_index=2; %initialize as integers
% min_grad_row
% min_grad_col
if SensorValue(min_grad_row) > SensorValue(min_grad_col)
    master_robot_index = min_grad_row;
    leader_robot_index= min_grad_col;
elseif SensorValue(min_grad_row) < SensorValue(min_grad_col)
    master_robot_index = min_grad_col;
    leader_robot_index= min_grad_row;
else
    master_robot_index = min_grad_row;
    leader_robot_index= min_grad_col;
end

% use bearing angles to determine if remaining robots are to the right or
% left of the master-leader bearing

master_leader_bearing= O(master_robot_index, leader_robot_index);
mlb_flip= master_leader_bearing + pi; % add 180 degrees to form "two halves" of circle
ml_bearing= master_leader_bearing; 

if mlb_flip >=pi    %simplifies math logic in lines 143-170
    mlb_flip= mod(mlb_flip, 2*pi);
else
    mlb_flip=mlb_flip;
end

num_robots_left= 0; % set initial counts for number of robots to 0
num_robots_right= 0;

for i= 1:length(x)
    if i ~= master_robot_index && i~=leader_robot_index
        mr_bearing= O(master_robot_index,i);  % master-robot_i bearing
        if master_leader_bearing >= pi
            if mr_bearing >0 && mr_bearing< mlb_flip
                num_robots_left= num_robots_left +1;
            elseif mr_bearing>mlb_flip && mr_bearing < master_leader_bearing
                num_robots_right= num_robots_right +1;
            elseif mr_bearing> ml_bearing && mr_bearing < 2*pi
                num_robots_left= num_robots_left+1;
            else
                num_robots_left= num_robots_left;
                num_robots_right= num_robots_right; % possibly an error
            end
        elseif master_leader_bearing<pi
            if mr_bearing>0 && mr_bearing<master_leader_bearing
                num_robots_right= num_robots_right + 1;
            elseif mr_bearing>ml_bearing && mr_bearing < mlb_flip
                num_robots_left= num_robots_left + 1;
            elseif mr_bearing > mlb_flip && mr_bearing < 2*pi
                num_robots_right= num_robots_right+1;
            else
                num_robots_left= num_robots_left;
                num_robots_right= num_robots_right;
            end
        end
    end
end

%note that the above doesn't give us info about WHICH robots are left and
%right, but is just a check that there exists a robot to the left/right
%% Using information about robot roles, determine "Swarm State"
%% -- cut into different Matlab Function in Simulink ???
sensor_val_mast_flag= 0;
roles_filled_flag = 0;

%check if the sensor value of the "master" is the highest of the local
%group and change flag to 1 if true
if SensorValue(master_robot_index) == max(SensorValue)
    sensor_val_mast_flag = 1;
else
    sensor_val_mast_flag= 0;
end

%check if all roles filled and change the flag to 1 if true
if num_robots_left>0 && num_robots_right>0
    roles_filled_flag = 1;
else
    roles_filled_flag = 0;
end

if sensor_val_mast_flag && roles_filled_flag
    SwarmStateNum= 1; % 'On ridge'
elseif roles_filled_flag && ~sensor_val_mast_flag
    SwarmStateNum= 2; % 'off ridge'
elseif sensor_val_mast_flag && ~roles_filled_flag
    if num_robots_left==0
        SwarmStateNum = 3 ;%  'offset right'
    elseif num_robots_right==0
        SwarmStateNum= 4 ;%'offset left'
    else
        SwarmStateNum= 5 ;% 'invalid-roles_filled_flag' %shouldn't occur
    end
else
    SwarmStateNum= 6; % 'invalid- general case failure'
end

%% Based on swarm state, enter a "behavior mode"

if SwarmStateNum==1
    % swarm is on ridge, follow the master/leader bearing line
    Vfx= cos(master_leader_bearing);
    Vfy= sin(master_leader_bearing);
    Vft= 0;
    Vf= [Vfx Vfy Vft];
elseif SwarmStateNum ==2
    % swarm is off the ridge, enter find max behavior
    [Vf] = SwarmSimFindMax(RobotParams,NRobot,SensorRange,ScalarFieldSelection);
elseif SwarmStateNum ==3
    % swarm is offset to the right, "partitioned" behavior
    if NRobot== master_robot_index
        Vfx= cos(master_leader_bearing);
        Vfy= sin(master_leader_bearing);
        Vft= 0;
        Vf= [Vfx Vfy Vft];
    else
        rotated_bearing= master_leader_bearing + (pi/4);
        Vfx= cos(rotated_bearing);
        Vfy= sin(rotated_bearing);
        Vft= 0;
        Vf= [Vfx Vfy Vft];
    end
elseif SwarmStateNum ==4
    % swarm is offset to the left, "partitioned" behavior
    if NRobot== master_robot_index
        Vfx= cos(master_leader_bearing);
        Vfy= sin(master_leader_bearing);
        Vft= 0;
        Vf= [Vfx Vfy Vft];
    else
        rotated_bearing= master_leader_bearing - (pi/4);
        Vfx= cos(rotated_bearing);
        Vfy= sin(rotated_bearing);
        Vft= 0;
        Vf= [Vfx Vfy Vft];
    end
elseif SwarmStateNum ==5
    % something is weird, enter find max behavior
    [Vf] = SwarmSimFindMax(RobotParams,NRobot,SensorRange,ScalarFieldSelection);
elseif SwarmStateNum == 6
    % shit is fucked, enter find max behavior
    [Vf] = SwarmSimFindMax(RobotParams,NRobot,SensorRange,ScalarFieldSelection);
else
    %shit is really fucked, enter find max behavior
    [Vf] = SwarmSimFindMax(RobotParams,NRobot,SensorRange,ScalarFieldSelection);
end

Vf= Vf;
% SwarmStateNum 
% num_robots_right 
% num_robots_left 
undelcared_varible;
end




