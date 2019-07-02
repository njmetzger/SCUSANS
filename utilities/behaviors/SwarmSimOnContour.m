function [contourState] = SwarmSimOnContour(RobotParams, NRobot, SensorRange, DesiredValue,CONTOUR_BUFFER)
% SWARMSIMONCOUNTER - <Determines if a robot is above, below, or on a
% desired contour value.>

% Outputs:
%   contourState     1=below contour, 2= above contour, 3= on contour

%% Initialize Variables

% Determine number of robots based off length of robot params vector 
N= floor(length(RobotParams)/4); 

%% initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
x=zeros(1,N);               
y=zeros(1,N);
theta=zeros(1,N);
SensorValue= zeros(1,N); % this is the value of the "sensor reading" from each robot's "on-board" sensor. 
contourState=1;
onContour=false;
belowContour=false;
aboveContour=false;

%% Set x,y,theta, and SensorValue inputs into an array
x(1,1:N)=RobotParams(1:4:4*N);
y(1,1:N)=RobotParams(2:4:4*N);
theta(1,1:N)=RobotParams(3:4:4*N);
SensorValue(1,1:N)=RobotParams(4:4:4*N);

%% Algorithm

% Grab robot's sensor reading
robotValue=0; 
robotValue=SensorValue(NRobot); 

% Check to see if robot is close to desired contour, below, or above
% contour
if abs(robotValue - DesiredValue)< CONTOUR_BUFFER
    onContour=true;
    contourState=3;
    return;
elseif robotValue <= DesiredValue
    belowContour=true;
    contourState=1;
    return;
elseif  robotValue > DesiredValue
    aboveContour=true;
    contourState=2;
    return;
else
    aboveContour=true;
    contourState=2;
    return;
end

end