function [Vf] = SwarmSimFindMax(RobotParams, NRobot, SensorRange)
%SwarmSimFindMax Find Max behavior called in Find Max block of Swarm_Robot_Base
%        MOST RECENT UPDATE: 09/18/2018 by NJM
%   SwarmSimFindMax takes the inputs of RobotParams, NRobot, and Sensor
%   Range and outputs the resultant velocity. Individual velocity of robot
%   is determined by comparing "Sensor Value" to surrounding robots and
%   moving in the direction of higher readings.

%% Initialize Variables

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4); 

%% initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N); % this is the value of the "sensor reading" from each robot's "on-board" sensor. 
% d - distance between "self" and other robot
% O - bearing angle between "self" and other robot
d=zeros(1,N);
O=zeros(1,N);
x_comp=zeros(1,N);
y_comp=zeros(1,N);
amp=zeros(1,N);
% [Vx, Vy, Vt] = velocity in x direction, velocity in y direction,
% rotational velocity about the z-axis 
%Vx=zeros(1,N);
%Vy=zeros(1,N);
%Vt=0;
%Vfx, Vfy, Vft are final values of Vx, Vy, Vt
% Vfx=0;
% Vfy=0;
Vft=0;
% Vf - final velocity as a vector.
Vf=[0.0 0.0 0.0];
% 
% x_comp=zeros(1,N); 
% y_comp=zeros(1,N);

%% Set x,y,theta, and SensorValue inputs into an array
x=RobotParams(1:4:end);
y=RobotParams(2:4:end);
theta=RobotParams(3:4:end);
SensorValue=RobotParams(4:4:end);

%% Find Min/Max

% Determine distance and angle to each robot 
d = sqrt(( x(NRobot)-x).^2 + (y(NRobot)-y).^2 );
O = atan2((y-y(NRobot)),(x-x(NRobot)));
amp = SensorValue-SensorValue(NRobot);

% max_idx=find(robotAmp==max(robotAmp));

inRange_idx=find(d<=SensorRange);

x_comp=cos(O(inRange_idx)).*amp(inRange_idx);
y_comp=sin(O(inRange_idx)).*amp(inRange_idx);

Vfx=sum(x_comp);
Vfy=sum(y_comp);

% Convert the sums into a vector that is then passed to the robot:
Vf= [Vfx(1) Vfy(1) Vft];

end
