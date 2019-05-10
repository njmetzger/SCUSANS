function [Vf,onRidge]= SwarmSimFollowTrench(RobotParams, NRobot,SensorRange,TrenchBuffer)
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

x(1,1:N)=RobotParams(1:4:end);
y(1,1:N)=RobotParams(2:4:end);
theta(1,1:N)=RobotParams(3:4:end);
SensorValue(1,1:N)=RobotParams(4:4:end);

% Determine distance and angle to each robot
d= sqrt( ( x(NRobot)-x ).^2 + ( y(NRobot)-y ).^2 );
O= atan2((y-y(NRobot)),(x-x(NRobot)));


%% Algorithm

inRange_idx=find(d<=SensorRange);

% Find robot with min sensor value
if length(inRange_idx) ~=0
    min_robot_idx=find(SensorValue==min(SensorValue(inRange_idx)));
    if length(min_robot_idx)>1
        min_robot_idx = min_robot_idx(1);
    end
    
    for i=1:N
        d_from_min(i) = sqrt( ( x(min_robot_idx)-x(i) )^2 + ( y(min_robot_idx)-y(i) )^2 );
        delta_z_from_min(i) = SensorValue(i)- SensorValue(min_robot_idx);
        if delta_z_from_min(i) < TrenchBuffer
            amp(i)=0;
        else
            amp(i)= d_from_min(i)/delta_z_from_min(i);
        end
    end
    % Find max "amplitude" robot
    trench_robot_idx=find(amp==max(amp(d<=SensorRange)));
    if length(trench_robot_idx)>1
        trench_robot_idx = trench_robot_idx(1);
    end
    if ~isempty(trench_robot_idx) && length(trench_robot_idx) == 1
        % Angle from min to ridge robot
        % O_ridge(1) = atan2((y(trench_robot_idx)-y(min_robot_idx)),(x(trench_robot_idx)-x(min_robot_idx)));
        % Now calculate vector from min robot to trench robot
        
        %calculate angle from master to All robots an
        for i = 1:N
            O_minToRobots(i) = atan2((y(i)-y(min_robot_idx)),(x(i)-x(min_robot_idx)));
        end
        Theta_d = O_minToRobots(trench_robot_idx);
        NIR = length(inRange_idx);
        if Theta_d < 0
            %Selects all robots to the left ie from theta_d to pi+theta d
            %(removing max robot with angle of 0
            n_robots_left = sum((Theta_d*ones(1,NIR)<O_minToRobots(inRange_idx)) & (O_minToRobots(inRange_idx)<= (pi+Theta_d)*(ones(1,NIR)))) -1;
            %selects all robots to the right ie from pi+theta_d to pi and
            %robots from -pi to theta_d
            n_robots_right = sum(((Theta_d+pi)*ones(1,NIR) < O_minToRobots(inRange_idx)) | (O_minToRobots(inRange_idx) < Theta_d*ones(1,NIR)));
        else
            %selects all robots to the right ie from theta_d-pi to theta_d
            %removes max robot with angle of 0
            n_robots_right = sum(((Theta_d-pi)*ones(1,NIR) <= O_minToRobots(inRange_idx)) & (O_minToRobots(inRange_idx) < Theta_d*ones(1,NIR)))-1;
            %selects all robots to the left ie from theta_d to pi or -pi to
            %(theta_d -pi)
            n_robots_left = sum((Theta_d*ones(1,NIR)<O_minToRobots(inRange_idx)) | (O_minToRobots(inRange_idx)< (Theta_d-pi)*ones(1,NIR)));
        end
        N_sided = floor((NIR-2)/2)-1;
        if N_sided < 2
            switch NRobot
                case min_robot_idx
                    Vfx=0;
                    Vfy=0;
                otherwise
                    Vx=x(min_robot_idx)-x(NRobot);
                    Vy=x(min_robot_idx)-x(NRobot);
                    Vfx = Vx/sqrt(sum(Vx^2+Vy^2));
                    Vfy = Vy/sqrt(sum(Vx^2+Vy^2));
            end
        elseif n_robots_right>=N_sided && n_robots_left>=N_sided
            Vfs = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx)];
            Vm = sqrt(sum(Vfs.^2));
            Vfx=Vfs(1)/Vm;
            Vfy=Vfs(2)/Vm;
        elseif n_robots_right>N_sided
            MR = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx),0];
            V = cross(MR ,[0,0,-1]);
            Vfs = V(1:2);
            Vm = sqrt(sum(Vfs.^2));
            Vfx=Vfs(1)/Vm;
            Vfy=Vfs(2)/Vm;
        elseif n_robots_left> N_sided
            MR = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx),0];
            V = cross(MR ,[0,0,1]);
            Vfs = V(1:2);
            Vm = sqrt(sum(Vfs.^2));
            Vfx=Vfs(1)/Vm;
            Vfy=Vfs(2)/Vm;
        else
            disp('Ridge following error')
        end
        
        
    else
        Vfx=0;
        Vfy=0;
    end
    Vf = [Vfx(1) Vfy(1) Vft(1)];
    
end
