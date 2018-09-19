function [Vf] = SwarmSimAttract(RobotParams, NRobot, SensorRange)
%SwarmSimAttract Attract behavior for Swarm_Adaptive_Navigation_Simulator.slx
%   LATEST UPDATE: 08/30/2018 by NJM 
% This attract behavior is called by the Robot # behavior blocks in
%   Swarm_Adaptive_Naviagtion_Simulator/Attract blocks. The attract behavior
%   causes the robots to clump together. It is set for constant velocity
%   and for constant force. For example, robots feel same attractive force
%   to each other when they are 10 units away or 1 unit away. Attraction is
%   based on average angle between "self" and other robots that are within
%   Sensor Range. 


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
Vx=zeros(1,N);
Vy=zeros(1,N);
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

AttractAngle= 0; 

%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Attract Behavior 
V_const=3; 

% Determine distance and angle to each robot 
for i=1:N 
    d(i) = sqrt(abs(x(NRobot)-x(i))^2+abs(y(NRobot)-y(i))^2);
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
    Vx(i)= V_const*cos(O(i));
    Vy(i)= V_const*sin(O(i));
end

% If robot is within sensor range of  Nrobot, it is attracted to that
% robot. If the  robot is outside of sensor range, then that robot has
% no effect on NRobot's velocity. 

% for i= 1:N
%     % if distance is greater than allowable sensor range, set angle
%     % contribution to 0
%     if d(i) > SensorRange
%         O(i) = 0;
%     % if distance is equal to 0 - as is the case for NRobot to NRobot - set
%     % angle contribution to 0
%     elseif d(i) == 0
%         O(i) = 0; 
%     else 
%         O(i)=O(i); 
%     end
% end
% Now that angle to all other robots has been determined, move in the
% direction of the average angle. 
idx=find(d<=SensorRange);

% %determine the reduced vector of angles that does not consider 0 values: 
% relevant_angles = O(O~=0); 
% 
% % attraction angle is the average of the relevant angles 
% AttractAngle = mean(relevant_angles);

%Discretize the commanded angle 
%discreteAngles=linspace(-2*pi,2*pi,20); 
%diffs= bsxfun(@minus,AttractAngle,discreteAngles); 
%[~, inds]= min(abs(diffs),[],2); 
%angle=discreteAngles(inds);

%Calculate velocity vector components: 
Vfx= sum(Vx(idx));
Vfy= sum(Vy(idx)); 

Vf= [Vfx Vfy Vft];


end 

