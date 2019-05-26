function Vf= SwarmSimFollowTrench(RobotParams, NRobot,SensorRange)
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
O_minToRobots = zeros(1,N);
d_from_min=zeros(1,N);  
delta_z_from_min=zeros(1,N); 
amp=zeros(1,N); 
% ridgeVector=[0.0 0.0 0.0];
Vfx=0.0;
Vfy=0.0;
Vft=0.0;
Vf = [0.0 0.0 0.0];



%% Set x,y,theta, and SensorValue inputs into an array

x(1,1:N)=RobotParams(1:4:4*N);
y(1,1:N)=RobotParams(2:4:4*N);
theta(1,1:N)=RobotParams(3:4:4*N);
SensorValue(1,1:N)=RobotParams(4:4:4*N);

% Determine distance and angle to each robot  
d = sqrt( ( x(NRobot)-x ).^2 + ( y(NRobot)-y ).^2 ); 
O = atan2((y-y(NRobot)),(x-x(NRobot)));

%% Algorithm

inRange_idx=find(d<=SensorRange);

% Find robot with max sensor value
min_robot_idx=find(SensorValue==min(SensorValue(inRange_idx)));

% Calculate weighting function 
for i=1:N
        d_from_min(i) = sqrt( ( x(min_robot_idx)-x(i) )^2 + ( y(min_robot_idx)-y(i) )^2 );
        delta_z_from_min(i) = SensorValue(i)-SensorValue(min_robot_idx);
        amp(i)= d_from_min(i)/delta_z_from_min(i);
end

% Find min"amplitude" robot
ridge_robot_idx=find(amp==max(amp(inRange_idx)));
if ~isempty(ridge_robot_idx) && length(ridge_robot_idx) == 1 
    % Angle from min to ridge robot 
    % O_ridge(1) = atan2((y(ridge_robot_idx)-y(min_robot_idx)),(x(ridge_robot_idx)-x(min_robot_idx)));
    % Now calculate vector from min robot to ridge robot 

    %calculate angle from master to All robots an
    for i = 1:N 
       O_minToRobots(i) = atan2((y(i)-y(min_robot_idx)),(x(i)-x(min_robot_idx)));
    end
    Theta_d = O_minToRobots(ridge_robot_idx); 
    if Theta_d < 0 
        n_robots_left = sum((Theta_d*ones(1,N)<O_minToRobots) & (O_minToRobots<= (pi+Theta_d)*(ones(1,N)))); 
        n_robots_right = sum(((Theta_d+pi)*ones(1,N) < O_minToRobots) | (O_minToRobots < Theta_d*ones(1,N))); 
    else
        n_robots_right = sum(((Theta_d-pi)*ones(1,N) <= O_minToRobots) & (O_minToRobots < Theta_d*ones(1,N))); 
        n_robots_left = sum((Theta_d*ones(1,N)<O_minToRobots) | (O_minToRobots< (Theta_d-pi)*ones(1,N))); 
    end
    if n_robots_right>0 && n_robots_left>0
        Vf = [x(ridge_robot_idx)-x(min_robot_idx), y(ridge_robot_idx)-y(min_robot_idx)];
        Vm = sqrt(sum(Vf.^2));
        Vfx=Vf(1)/Vm;
        Vfy=Vf(2)/Vm;
    elseif n_robots_right>0
        switch NRobot
            case min_robot_idx
                Vfx = 0;
                Vfy = 0;
            otherwise
                MR = [x(ridge_robot_idx)-x(min_robot_idx), y(ridge_robot_idx)-y(min_robot_idx),0];
                V = cross(MR ,[0,0,-1]);
                Vf = V(1:2);
                Vm = sqrt(sum(Vf.^2));
                Vfx=Vf(1)/Vm;
                Vfy=Vf(2)/Vm;
        end
    elseif n_robots_left>0 
        switch NRobot
            case min_robot_idx
                Vfx = 0;
                Vfy = 0; 
            otherwise
                MR = [x(ridge_robot_idx)-x(min_robot_idx), y(ridge_robot_idx)-y(min_robot_idx),0];
                V = cross(MR ,[0,0,1]);
                Vf = V(1:2); 
                Vm = sqrt(sum(Vf.^2));
                Vfx=Vf(1)/Vm;
                Vfy=Vf(2)/Vm;
        end
    else
        disp('Trench following error')
    end
    
else
    Vfx=0;
    Vfy=0;
end
Vf = [Vfx Vfy Vft]; 

end