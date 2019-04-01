function Vf= SwarmSimFollowRidge(RobotParams, NRobot,SensorRange)
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
O_maxToRobots = zeros(1,N);
d_from_max=zeros(1,N);  
delta_z_from_max=zeros(1,N); 
amp=zeros(1,N); 
% ridgeVector=[0.0 0.0 0.0];
Vfx=0.0;
Vfy=0.0;
Vft=0.0;
Vf = [0.0 0.0 0.0];



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
    if max_robot_idx ~= i
        d_from_max(i) = sqrt( ( x(max_robot_idx)-x(i) )^2 + ( y(max_robot_idx)-y(i) )^2 );
        delta_z_from_max(i) = SensorValue(max_robot_idx)-SensorValue(i);
        amp(i)= d_from_max(i)/delta_z_from_max(i);
    else
        amp(i) = -inf; 
    end
end

% Find max "amplitude" robot
ridge_robot_idx=find(amp==max(amp(inRange_idx)));
if ~isempty(ridge_robot_idx) && length(ridge_robot_idx) == 1 
    % Angle from max to ridge robot 
    % O_ridge(1) = atan2((y(ridge_robot_idx)-y(max_robot_idx)),(x(ridge_robot_idx)-x(max_robot_idx)));
    % Now calculate vector from max robot to ridge robot 

    %calculate angle from master to All robots an
    for i = 1:N 
        if i ==max_robot_idx
           O_maxToRobots(i) = nan; 
        else
           O_maxToRobots(i) = atan2((y(i)-y(max_robot_idx)),(x(i)-x(max_robot_idx)));
        end
    end
    Theta_d = O_maxToRobots(ridge_robot_idx); 
    if Theta_d < 0 
        n_robots_left = sum((Theta_d*ones(1,N)<O_maxToRobots) & (O_maxToRobots<= (pi+Theta_d)*(ones(1,N)))); 
        n_robots_right = sum(((Theta_d+pi)*ones(1,N) < O_maxToRobots) | (O_maxToRobots < Theta_d*ones(1,N))); 
    else
        n_robots_right = sum(((Theta_d-pi)*ones(1,N) <= O_maxToRobots) & (O_maxToRobots < Theta_d*ones(1,N))); 
        n_robots_left = sum((Theta_d*ones(1,N)<O_maxToRobots) | (O_maxToRobots< (Theta_d-pi)*ones(1,N))); 
    end
    if n_robots_right>0 && n_robots_left>0
        Vf = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx)];
        Vm = sqrt(sum(Vf.^2));
        Vfx=Vf(1)/Vm;
        Vfy=Vf(2)/Vm;
    elseif n_robots_right>0
        switch NRobot
            case max_robot_idx
                Vfx = 0;
                Vfy = 0;
            otherwise
                MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
                V = cross(MR ,[0,0,-1]);
                Vf = V(1:2);
                Vm = sqrt(sum(Vf.^2));
                Vfx=Vf(1)/Vm;
                Vfy=Vf(2)/Vm;
        end
    elseif n_robots_left>0 
        switch NRobot
            case max_robot_idx
                Vfx = 0;
                Vfy = 0; 
            otherwise
                MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
                V = cross(MR ,[0,0,1]);
                Vf = V(1:2); 
                Vm = sqrt(sum(Vf.^2));
                Vfx=Vf(1)/Vm;
                Vfy=Vf(2)/Vm;
        end
    else
        disp('Ridge following error')
    end
    
else
    Vfx=0;
    Vfy=0;
end
Vf = [Vfx Vfy Vft]; 

end