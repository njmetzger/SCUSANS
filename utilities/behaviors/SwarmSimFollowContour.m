function [Vf] = SwarmSimFollowContour(RobotParams, NRobot, SensorRange, DesiredValue)
% SWARMSIMFOLLOWCONTOUR - <Attractive function to create contour following behavior>

% Description:
%   This function follows the same base logic as the "FindMin/FindMax"
%   behaviors but applies a 90 degree rotation to the output velocity
%   vector. 

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
Vft=0;
% Vf - final velocity as a vector.
Vf=[0.0 0.0 0.0];

%% Set x,y,theta, and SensorValue inputs into an array
x(1,1:N)=RobotParams(1:4:4*N);
y(1,1:N)=RobotParams(2:4:4*N);
theta(1,1:N)=RobotParams(3:4:4*N);
SensorValue(1,1:N)=RobotParams(4:4:4*N);

%% Find Min/Max

% Determine distance and angle to each robot 
d = sqrt(abs(x(NRobot)-x).^2+abs(y(NRobot)-y).^2);
O = atan2((y-y(NRobot)),(x-x(NRobot)));
amp=(SensorValue-SensorValue(NRobot));

% max_idx=find(robotAmp==max(robotAmp));

inRange_idx=find(d<=SensorRange);

x_comp=cos(O(inRange_idx)).*amp(inRange_idx);
y_comp=sin(O(inRange_idx)).*amp(inRange_idx);

Vx=sum(x_comp);
Vy=sum(y_comp);

% Recalculate unit vectors for 90 degree rotation
Vfy=(Vx^2+Vy^2)^(.5)*sin(atan2(Vy,Vx) + pi/2);
Vfx=(Vx^2+Vy^2)^(.5)*cos(atan2(Vy,Vx) + pi/2);

% Convert the sums into a vector that is then passed to the robot:
Vf= [Vfx Vfy Vft];

end
