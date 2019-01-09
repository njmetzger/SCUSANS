function [ridgeState, Vfx, Vfy] = SwarmSimRidgeCheck(RobotParams, NRobot, SensorRange)
% SWARMSIMCHECKRIDGE - <Determines....>

% Outputs:
%   ridgeState      1=maxRobot, 2= targetRobot, 3= offRidge_right 4=
%                                                           offRidge_left
%   maxRobot        TRUE if on contour
%   targetRobot     TRUE if below contour
%   offRidge        TRUE if above contour
%% Initialize Variables

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4); 

%% initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N); % this is the value of the "sensor reading" from each robot's "on-board" sensor. 
d=zeros(1,N);
O=zeros(1,N);
d_from_max=zeros(1,N);  
delta_z_from_max=zeros(1,N); 
amp=zeros(1,N); 
% ridgeVector=[0.0 0.0 0.0];
Vfx=0.0;
Vfy=0.0;

ridgeState=4;
O_ridge=[pi 0];
O_robotToRidge=0;
O_maxToRidge=0;



%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i*4);
end

% Determine distance and angle to each robot 
for i=1:N 
    d(i) = sqrt( ( x(NRobot)-x(i) )^2 + ( y(NRobot)-y(i) )^2 ); 
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
end

%% Algorithm

inRange_idx=find(d<=SensorRange);

% Find robot with max sensor value
max_robot_idx=find(SensorValue==max(SensorValue(inRange_idx)));

% Calculate weighting function 
for i=1:N
    d_from_max(i) = sqrt( ( x(max_robot_idx)-x(i) )^2 + ( y(max_robot_idx)-y(i) )^2 );
    delta_z_from_max(i) = SensorValue(max_robot_idx)-SensorValue(i);
    amp(i)= 5*d_from_max(i)/delta_z_from_max(i);
end

% Find max "amplitude" robot
ridge_robot_idx=find(amp==max(amp(inRange_idx)));
if ~isempty(ridge_robot_idx)
    % Angle from max to ridge robot 
    % O_ridge(1) = atan2((y(ridge_robot_idx)-y(max_robot_idx)),(x(ridge_robot_idx)-x(max_robot_idx)));
    % Now calculate vector from max robot to ridge robot 
    O_maxToRidge = atan2((y(ridge_robot_idx)-y(max_robot_idx)),(x(ridge_robot_idx)-x(max_robot_idx)));

    O_robotToRidge = atan2((y(ridge_robot_idx)-y(NRobot)),(x(ridge_robot_idx)-x(NRobot)));


    % Check to see if current robot is the max robot 
    if NRobot==max_robot_idx
        ridgeState = 1;
    end

    % Check to see if current robot is the ridge robot 
    if NRobot==ridge_robot_idx
        ridgeState = 2;
    end

    if O_robotToRidge>=O_maxToRidge
        ridgeState = 3;
    end

    % Now calculate vector from max robot to ridge robot 
    O_maxToRidge = atan2((y(ridge_robot_idx)-y(max_robot_idx)),(x(ridge_robot_idx)-x(max_robot_idx)));
    Vfx= 100.*cos(O_maxToRidge);
    Vfy= 100.*sin(O_maxToRidge);

    Vfx=Vfx(1);
    Vfy=Vfy(1);
else
    Vfx=0;
    Vfy=0;
end



end