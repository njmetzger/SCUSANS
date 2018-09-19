function [Vf] = SwarmSimFindMax(RobotParams, NRobot, SensorRange)
%SwarmSimFindMax Find Max behavior called in Find Max block of Swarm_Robot_Base
%        MOST RECENT UPDATE: 09/18/2018 by NJM
%   SwarmSimFindMax takes the inputs of RobotParams, NRobot, and Sensor
%   Range and outputs the resultant velocity. Individual velocity of robot
%   is determined by comparing "Sensor Value" to surrounding robots and
%   moving in the direction of higher readings.

%% Initialize Variables

% Determine number of robots

N= floor(length(RobotParams)/4);

% Robot pose variables -
    % (x,y)- position of robot in global frame
    % theta- rotation of robot about global z-axis
    % SensorValue- value read from SensorValue function

x= zeros(1,N);
y= zeros(1,N);
theta= zeros(1,N);
SensorValue= zeros(1,N);

% variables used in Find Min calculation
d=zeros(1,N);
O=zeros(1,N);
val_diff=zeros(1,N);
x_off=zeros(1,N);
y_off=zeros(1,N);
Vx=zeros(1,N);
Vy=zeros(1,N);
Vt=0;
Vfx=0;
Vfy=0;
Vft=0;
Vf=[0.0 0.0 0.0];

% robotAmp=zeros(1,N);
%x_comp=zeros(1,N);
%y_comp=zeros(1,N);

%% Set x, y, theta inputs into array
for i=1:N
    x(i)=RobotParams(i*4-3);
    y(i)=RobotParams(i*4-2);
    theta(i)=RobotParams(i*4-1);
    SensorValue(i)=RobotParams(i);
end

%% Find Max Behavior

% determine distance, angle, and value difference from robot to every other robot

for i= 1:N
    d(i) = sqrt(abs(x(NRobot)-x(i))^2+abs(y(NRobot)-y(i))^2);
    O(i) = atan2((y(i)-y(NRobot)),(x(i)-x(NRobot)));
    % determine value difference FROM self TO other robot.
    % + value means own value is higher, - value means own value is lower
    val_diff(i)= SensorValue(NRobot) - SensorValue(i);
end

% determine unit vectors FROM self TO other robots using angle

for i=1:N
    if O(i) == 0
        x_off(i)=0;
        y_off(i)=0;
    else
        x_off(i)= cos(O(i));
        y_off(i)= sin(O(i));
    end
end

% multiply offsets by MINUS val_diff to get x and y direction of motion, scaled
% by how different the values are. Sign of val_diff determines motion up or
% down gradient, magnitude of val_diff will scale the velocity vector so
% robot is more affected by larger sensor reading differences.

for i=1:N
    % if the robots are out of sensor range, then the velocity contribution is 0  
    if d(i) > SensorRange
%         Vx(i) = 0;
%         Vy(i) = 0;
            Vx(i) = NaN;
            Vy(i) = NaN; 
    else
        Vx(i)= -val_diff(i)*x_off(i);
        Vy(i)= -val_diff(i)*y_off(i);
    end 
end

% determine bearing angle by taking the sum of the Vx and Vy terms 

Vx_total=sum(Vx(isnan(Vx)~=1)); 
Vy_total=sum(Vy(isnan(Vy)~=1)); 
BearingAngle= atan2(Vy_total,Vx_total);

% Determine Vfx and Vfy based off bearingangle 

V_Constant=1; 

Vfx= V_Constant*cos(BearingAngle);
Vfy= V_Constant*sin(BearingAngle); 

Vf= [Vfx, Vfy, Vft];

end
