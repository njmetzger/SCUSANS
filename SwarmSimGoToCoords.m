function Vf = SwarmSimGoToCoords(RobotParams, NRobot,GoalX, GoalY)

%SwarmSimGoToCoords Go to behavior called in Go Too Coords block of Swarm_Robot_Base
%        MOST RECENT UPDATE: 09/18/2018 by NJM
%   SwarmSimGoToCoords takes the inputs of RobotParams and NRobot and outputs the resultant velocity. Individual velocity of robot
%   is determined by comparing the NRobot's corrent position to the goal
%   position. 

%% Initialize Variables

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4); 

%% initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N); % this is the value of the "sensor reading" from each robot's "on-board" sensor. 
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
Vf=[0.0 0.0 0.0]';
% 
% x_comp=zeros(1,N); 
% y_comp=zeros(1,N);

%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i*4);
end

%% Find Global Hedding to Go TO  
Vx = GoalX - x(NRobot);
Vy = GoalY - y(NRobot);
V_mag =sqrt(Vx^2 + Vy^2); 
Vfx = Vx(1)/V_mag;
Vfy = Vy(1)/V_mag;
% Convert the sums into a vector that is then passed to the robot:
Vf(1)= Vfx; 
Vf(2)= Vfy; 
Vf(3)= Vft;

end