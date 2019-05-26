function [Vf] = SwarmSimDisperse(RobotParams, NRobot, SensorRange)
%SwarmSimDisperse Disperse behavior for Swarm_Adaptive_Navigation_Simulator.slx
%   LATEST UPDATE: 09/04/2018 by NJM 
% This disperse behavior is called by the Robot # behavior blocks in
%   Swarm_Adaptive_Naviagtion_Simulator/Disperse blocks. The disperse behavior
%   causes the robots to clump together. It is set for constant velocity
%   and for constant force. For example, robots feel same repulsive force
%   to each other when they are 10 units away or 2 unit away. Repulsion is
%   based on average angle between "self" and other robots that are within
%   Sensor Range. NOTE that this is NOT the avoidance behavior for
%   collision avoidance between robots and avoidance of other objects. 

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4);

%establish a "stop distance" that is within the sensor range so that once
%robots are "dispersed" they are still within sensor range of each other 
stop_dist= SensorRange - 2 ; 

%% Initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N);
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

%% Set x,y,theta, and SensorValue inputs into an array
x=RobotParams(1:4:end);
y=RobotParams(2:4:end);
theta=RobotParams(3:4:end);
SensorValue=RobotParams(4:4:end);

%% Disperse Behavior 
 

% Determine distance and angle to each robot 

d = sqrt(( x(NRobot)-x).^2 + (y(NRobot)-y).^2 );
O = atan2((y-y(NRobot)),(x-x(NRobot)));
Vx= cos(O);
Vy= sin(O);


% If robot is within sensor range of  Nrobot, it is attracted to that
% robot. If the  robot is outside of sensor range, then that robot has
% no effect on NRobot's velocity. 

idx=find(d<=SensorRange);

%Discretize the commanded angle 
%discreteAngles=linspace(-2*pi,2*pi,20); 
%diffs= bsxfun(@minus,AttractAngle,discreteAngles); 
%[~, inds]= min(abs(diffs),[],2); 
%angle=discreteAngles(inds);

%Calculate velocity vector components: 
Vfx= sum(Vx(idx));
Vfy= sum(Vy(idx)); 

Vf= -[Vfx Vfy Vft];

end

