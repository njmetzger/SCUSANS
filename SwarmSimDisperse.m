function [Vf] = SwarmSimDisperse(RobotParams, NRobot, SensorRange)
%SwarmSimDisperse Disperse behavior for Swarm_Adaptive_Navigation_Simulator.slx
%   LATEST UPDATE: 09/17/2018 by NJM 
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
% [Vx, Vy, Vt] = velocity in x direction, velocity in y direction,
% rotational velocity about the z-axis 
%Vx=zeros(1,N);
%Vy=zeros(1,N);
%Vt=0;
%Vfx, Vfy, Vft are final values of Vx, Vy, Vt
Vfx=0;
Vfy=0;
Vft=0;
% Vf - final velocity as a vector.
Vf=[0.0 0.0 0.0];
% 
% x_comp=zeros(1,N); 
% y_comp=zeros(1,N);

%DisperseAngle= 0; 

%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end
%% Disperse Behavior 

% Determine distance and angle to each robot 
for i=1:N 
    d(i) = sqrt(abs(x(NRobot)-x(i))^2+abs(y(NRobot)-y(i))^2); 
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
end


% If a robot is within the "stop range" of NRobot, NRobot is repulsed by
% that robot. If a robot is outside of the "stop range," the robot has no
% effect on NRobot's velocity. 

for i= 1:N
    %if distance is greater than allowable stop range,set angle contribution to 0
    %if d(i) > stop_dist
        %O(i)= 0; 
    if d(i) > stop_dist
        O(i) = NaN; 
    %if distance is equal to 0 - case for NRobot to NRobot- set angle
    %contribution to 0 
    elseif d(i) == 0
        O(i) = NaN; 
    else 
        O(i) = O(i); 
    end 
end 

% Now that angle to all other robots has been determined, move in the
% direction of the average angle. 


%determine the reduced vector of angles that does not consider 0 values: 
relevant_angles = O(isnan(O)~=1);

% attraction angle is the average of the relevant angles 
AttractAngle= mean(relevant_angles);  

%Disperse angle is in opposite direction of attraction angle, so add pi
%radians to the determined attract angle 

%DisperseAngle= AttractAngle + pi; 

%want dispersion angle to be between 0 and 2*pi, so if the resultant angle is greater than 2*pi, subtract 2*pi from the value
% if DisperseAngle > 2*pi
%     DisperseAngle= DisperseAngle- 2*pi; 
% else 
%     DisperseAngle= DisperseAngle;  
% end 

%Discretize the commanded angle 
%discreteAngles=linspace(-2*pi,2*pi,20); 
%diffs= bsxfun(@minus,AttractAngle,discreteAngles); 
%[~, inds]= min(abs(diffs),[],2); 
%angle=discreteAngles(inds);

%Calculate velocity vector components 

% V_const=1; 
% Vfx= V_const*cos(DisperseAngle); 
% Vfy= V_const*sin(DisperseAngle); 
% 
% Vf= [Vfx Vfy Vft]; 

% because of an issue with Simulink (singularity in a transfer function?),
% logic was temporarily changed: 

V_const=1;
Vfx= V_const*cos(AttractAngle); 
Vfy= V_const*sin(AttractAngle); 

V_attract= [Vfx, Vfy, Vft];

Vf= -V_attract;


end

