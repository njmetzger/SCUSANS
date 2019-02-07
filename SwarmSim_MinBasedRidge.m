function [Vf] = SwarmSim_MinBasedRidge(RobotParams, NRobot, SensorRange, ScalarFieldSelection)
%SwarmSim_MinBasedRidge Simpler primitive-based attempt at ridge following
%   This ridge following algorithm attempts to implement a more
%   primitive-based method of ridge following. From a high level, this is a
%   combination of 1) On-ridge determination 2) Find Min 3) Attract 4)
%   Obstacle Avoidance 

%% Initialize Variables
% Determine number of robots based off length of robot params vector
N= floor(length(RobotParams)/4);
x=zeros(1,N);
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N);
d=zeros(N,N);
O=zeros(N,N);
Vfx= 0;
Vfy= 0;
Vft= 0;
Vf= [Vfx Vfy Vft];

%%  Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Ridge stuff- similar to what Max did before

% assign the "master" robot - robot with max sensor value 
master_robot_idx= find(SensorValue == max(SensorValue)); 

% compare values and determine the "leader" robot at the end of the
% shallowest gradient (i.e. the other robot on the ridge) 

% determine which robots are to the left or right of the master/leader
% bearing 

%% Ridge Check -- currently do this based off earlier checks
% may want to look at the 2nd derivative at some point? for a better
% mathetmatical definition of the ridge 


%% Output Vf 

% if on ridge, then enter a FindMin that only utilizes robots that are
% within a certain "buffer" bearing of the master-leader bearing

% if you're not on the ridge, then enter find max, utilizing information
% from all robots 

%% Attract and Obstacle Avoidance should still be on so that the swarm stays
% together, but so that it stays spaced out enough to keep definition of
% the ridge 

%% Adaptive Sizing stuff: 
% Obstacle Avoidance based off the localized gradients (with saturations to
% ensure no collisions and that robots stay in range of each other) 

% Contour buffer - when doing the contour following, possibly could make
% the contour buffer something that gets updated based off the local
% gradient readings (if its a very steep or shallow part of the field 

% Ridge buffer- if we have a very "skinny" ridge, would want to have a
% small angle buffer, and if we have a "fat" ridge, would want to have a
% wider angle ridge 



end

