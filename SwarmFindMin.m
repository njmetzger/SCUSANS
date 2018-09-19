function Vf = SwarmFindMin(RobotParams,NRobot, SensorRange)

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

%% Set x,y,theta, and SensorValue inputs into an array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Find Min/Max
V_const=3; 

% Determine distance and angle to each robot 
for i=1:N 
    d(i) = sqrt(abs(x(NRobot)-x(i))^2+abs(y(NRobot)-y(i))^2);
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
    amp(i)=(readScalarField(x(i),y(i))-readScalarField(x(NRobot),y(NRobot)));
end

% max_idx=find(robotAmp==max(robotAmp));

inRange_idx=find(d<=SensorRange);

for i=1:numel(inRange_idx)   
    x_comp(i)=cos(O(inRange_idx(i)))*amp(inRange_idx(i));
    y_comp(i)=sin(O(inRange_idx(i)))*amp(inRange_idx(i));
end

Vfx=sum(x_comp);
Vfy=sum(y_comp);

% Convert the sums into a vector that is then passed to the robot:
Vf= [Vfx Vfy Vft];
          
end 
