function [Vf] = SwarmSimObstacleAvoid(RobotParams, NRobot, SensorRange, AvoidRange)
%SwarmSimObstacleAvoid Obstacle Avoidance behavior for Swarm_Adaptive_Navigation_Simulator.slx
%   LATEST UPDATE: 09/06/2018 by NJM 
% This obstacle avoidance behavior is called by the Robot # behavior blocks in
%   Swarm_Adaptive_Naviagtion_Simulator/ObstacleAvoidance blocks. The
%   obstacle avoidance behavior causes robots to move away from other 
%   robots (or objects) that are sensed to be in the robot's avoidance
%   radius, which is an input for this function. The behavior is designed
%   to be non-linear, so that avoidance is damped outside of the
%   AvoidRange, but is the "strongest" component when the robot is close to
%   other robots or objects. 

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4);

%% Initialize Variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue=zeros(1,N);
% d - distance between "self" and other robot
% O - bearing angle between "self" and other robot
d=zeros(1,N);
O=zeros(1,N);
mag_velocity=zeros(1,N);
% [Vx, Vy, Vt] = velocity in x direction, velocity in y direction,
% rotational velocity about the z-axis 
Vx=zeros(1,N);
Vy=zeros(1,N);
Vt=0;
%Vfx, Vfy, Vft are final values of Vx, Vy, Vt
Vfx=0;
Vfy=0;
Vft=0;
% Vf - final velocity as a vector.
Vf=[0.0 0.0 0.0];
% 
% x_comp=zeros(1,N); 
% y_comp=zeros(1,N);

%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Obstacle Avoidance Behavior 

% Determine distance and angle to each robot 
for i=1:N 
    d(i) = ( ( x(NRobot)-x(i) )^2 + ( y(NRobot)-y(i) )^2 )^(0.5); 
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
end

% Calculate avoidance velocities 
for i=1:N
    if d(i)<=AvoidRange && d(i)~=0
        mag_velocity(i)=.5*1*(1/d(i)-1/AvoidRange)^4;
    else
        mag_velocity(i)=0;
    end
    
    if mag_velocity(i)>10000000
        mag_velocity(i)=10000000;
    end
    
    Vx(i)= mag_velocity(i)*cos(O(i)); 
    Vy(i)= mag_velocity(i)*sin(O(i));     
end

Vfx= sum(Vx(Vx~=0));
Vfy= sum(Vy(Vy~=0));

Vf= -[Vfx, Vfy, Vft]; 
end 
 

    

