function [Vf] = SwarmSimObstacleAvoid(RobotParams, NRobot, SensorRange, AvoidRange)
%SwarmSimObstacleAvoid Obstacle Avoidance behavior for Swarm_Adaptive_Navigation_Simulator.slx
%   LATEST UPDATE: 09/18/2018 by NJM 
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
    d(i) = sqrt(abs(x(NRobot)-x(i))^2+abs(y(NRobot)-y(i)))^2; 
    O(i) = atan2((y(NRobot)-y(i)),(x(NRobot)-x(i)));
end

%Determine the magnitude of the velocity based off non-linear, then use the
%angle between the two to determine Vx and Vy.

% To determine magnitude of velocity, set to 0 for the case that d=0
% (should only be the case for NRobot to NRobot). If this is not the case,
% then the magnitude of the velocity is set as (AvoidRange)^4/(distance^4).
% This gives non-linear behavior, with avoidance velocity dramatically
% increasing as the robots get closer than the "avoid range," which can be
% easily changed. 

for i=1:N 
    if abs(d(i)) < 0.01  %should only be case for NRobot to self - rather than being set to zero, is set 0.01 -- will be raised to the 4th power -- possible rounding error leading to singularity.
        mag_velocity= NaN; 
        Vx(i)= NaN;
        Vy(i)= NaN; 
    elseif d(i) > SensorRange %robots outside of Sensor Range do not affect velocity
        mag_velocity= NaN;
        Vx(i)= NaN;
        Vy(i)= NaN;
    else 
        mag_velocity= abs((AvoidRange^4)/(d(i)^4));
        Vx(i)= mag_velocity*cos(O(i)); 
        Vy(i)= mag_velocity*sin(O(i)); 
    end
    
 % to adapt to discretized angles, should be done at this step for each O(i). 
    
end 
    
% velocity is sum of all components of Vx that are not NaN
Vfx= sum(Vx(isnan(Vx)~=1)); 
Vfy= sum(Vy(isnan(Vy)~=1));

Vf= [Vfx, Vfy, Vft]; 
end 
 

    

