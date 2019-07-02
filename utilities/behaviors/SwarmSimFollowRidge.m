function [Vf,ridgeState]= SwarmSimFollowRidge(RobotParams, NRobot,SensorRange)
% SWARMSIMFOLLOWRIDGE - <Determines the vector Vf to follow in order for the swarm to follow a ridge>

% Outputs:
%   ridgeState      1=OnRidge, 2= offridge, 3= offRidge_right
%                   4= offRidge_left 5= Not enough robots
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
ridgeState = 0;



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

% Find robot with max sensor value
if length(inRange_idx) >=6
    max_robot_idx=find(SensorValue==max(SensorValue(inRange_idx)));
    if length(max_robot_idx)>1
        max_robot_idx = max_robot_idx(1);
    end
    RidgeBuffer = 0.1*(max(SensorValue(inRange_idx))-min(SensorValue(inRange_idx)));
    
    d_from_max = sqrt((x(max_robot_idx)*ones(size(x))-x ).^2 + ( y(max_robot_idx)*ones(size(x))-y).^2 );
    delta_z_from_max = SensorValue(max_robot_idx)*ones(size(SensorValue))-SensorValue;
    amp= (d_from_max.^2)./delta_z_from_max;
    amp(delta_z_from_max<RidgeBuffer)=0;
    % Find max "amplitude" robot
    alpha =1.0;
    ridge_robot_idxs=find(amp>=alpha*max(amp(inRange_idx)));
    R2rRight_b = 0;
    R2rLeft_b = 0;
    
    for n = 1:length(ridge_robot_idxs)
        ridge_robot_idx = ridge_robot_idxs(n);
        [Vfx_t,Vfy_t,ridgeState_t,R2rleft,R2rRight]= checkridge(max_robot_idx,ridge_robot_idx,x,y,SensorValue,N,NRobot,inRange_idx);
        if R2rleft>R2rLeft_b && R2rRight>R2rRight_b && ridgeState_t ==1
            R2rRight_b = R2rleft;
            R2rLeft_b =R2rleft ;
            Vfx= Vfx_t;
            Vfy=Vfy_t;
            ridgeState= ridgeState_t;
        elseif ridgeState ==0 || (ridgeState_t==3 & ridgeState==2) ||(ridgeState_t==4 & ridgeState==2)
            ridgeState = ridgeState_t;
            Vfx= Vfx_t;
            Vfy=Vfy_t;
        end
        
    end
    
    
else
    %Not enough robots inrange
    ridgeState= 5;
    max_robot_idx = find(SensorValue== max(SensorValue(inRange_idx)));
    if length(max_robot_idx)>1
        max_robot_idx = max_robot_idx(1);
    end
    switch NRobot
        case max_robot_idx
            Vfx=0;
            Vfy=0;
        otherwise
            Vx=x(max_robot_idx)-x(NRobot);
            Vy=y(max_robot_idx)-y(NRobot);
            Vfx = Vx/sqrt(sum(Vx.^2+Vy.^2));
            Vfy = Vy/sqrt(sum(Vx.^2+Vy.^2));
    end
    
end
if ridgeState ==2
    Vf = SwarmSimFindMax(RobotParams, NRobot, SensorRange);
    Vf = Vf./sqrt(sum(Vf.^2));
else
    Vf = [Vfx(1) Vfy(1) Vft(1)];
end
end
%% Helper Functions
function [Vfx,Vfy,ridgeState,R2rleft,R2rRight]= checkridge(max_robot_idx,ridge_robot_idx,x,y,SensorValue,N,NRobot,inRange_idx)
%Compute Homogeneous transform to look along vector from ridgeRobot
%to max robot to determine number and curvature of robots on either
%side of proposed ridge

%Find angles theta1 and theta2 for transform

theta1 = atan2(y(ridge_robot_idx) - y(max_robot_idx),x(ridge_robot_idx) - x(max_robot_idx));
theta2 = atan2(SensorValue(max_robot_idx)-SensorValue(ridge_robot_idx),sqrt((y(ridge_robot_idx) - y(max_robot_idx)).^2+(x(ridge_robot_idx) - x(max_robot_idx))^2));
%Build Transforms
R1 = [cos(theta1), sin(theta1),0;-sin(theta1),cos(theta1),0;0 0 1 ;];
P01 = [x(max_robot_idx);y(max_robot_idx); SensorValue(max_robot_idx)];
H1 = [R1 -R1*P01; 0 0 0  1];
R2 = [cos(theta2),0, -sin(theta2);0 1 0; sin(theta2),0,cos(theta2)];
H2 = [R2 [0;0;0]; 0 0 0 1];
H = H2*H1;
%Convert coordinates
RPa = zeros(N*4,1);
RPa(1:4:end)=x;
RPa(2:4:end)=y;
RPa(3:4:end)= SensorValue;
RPa(4:4:end) = 1;
ACell = repmat({H}, 1, N);
HM = blkdiag(ACell{:});
RPap = HM*RPa;
Xsp = RPap(1:4:end);
Ysp = RPap(2:4:end);
SVp = RPap(3:4:end);
Xsp = Xsp(inRange_idx);
Ysp = Ysp(inRange_idx);
SVp = SVp(inRange_idx);


%Determine if robots are on a ridge
nr_right = sum(Ysp<0);
nr_left = sum(Ysp>0);
nfr_right= sum(Ysp<0 & Xsp>=0); 
nfr_left = sum(Ysp>0 & Xsp>=0);

% Check if there aren't at least N_sided robots on one side
% Nsided must be at lest two
N_sided = floor(length(inRange_idx)/4);
if N_sided <2
    N_sided =2;
end
if nr_right <N_sided
    %Shift right
    MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
    V = cross(MR ,[0,0,1]);
    Vfs = V(1:2);
    Vm = sqrt(sum(Vfs.^2));
    Vfx=Vfs(1)/Vm;
    Vfy=Vfs(2)/Vm;
    ridgeState= 4;
    R2rleft =0;
    R2rRight=0;
elseif nr_left < N_sided
    %Shift left
    MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
    V = cross(MR ,[0,0,-1]);
    Vfs = V(1:2);
    Vm = sqrt(sum(Vfs.^2));
    Vfx=Vfs(1)/Vm;
    Vfy=Vfs(2)/Vm;
    ridgeState= 3;
    R2rleft =0;
    R2rRight=0;
else
    %There are at least two robots on either side of the ridge
    R2Limit= 0.5;
    if nfr_right >=2 && nfr_left>=2
        %Calculate slope of robots on either side of the ridge and R^2 values
        [Prr,Srr] = polyfit(Ysp(Ysp<=0 & Xsp>=0),SVp(Ysp<=0 & Xsp>=0),1);
        R2rRight = 1 - (Srr.normr/norm(SVp(Ysp<=0 & Xsp>=0) - mean(SVp(Ysp<=0 & Xsp>=0 ))))^2;
        [Prl,Srl] = polyfit(Ysp(Ysp>=0),SVp(Ysp>=0),1);
        R2rleft = 1 - (Srl.normr/norm(SVp(Ysp>=0) - mean(SVp(Ysp>=0 & Xsp>=0))))^2;
    else
        R2rleft =0;
        R2rRight=0;
        Prl =0;
        Prr=0;
    end
    
    if Prl(1) < 0 && Prr(1)>0 && R2rleft>=R2Limit && R2rRight >= R2Limit
        %Ridge with sufficient number of robots on either side
        Vfs = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx)];
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        ridgeState=1;
    elseif Prl(1) < 0 && Prr(1)>0 && R2rleft< R2Limit && R2rRight >= R2Limit
        %Shift left
        MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
        V = cross(MR ,[0,0,-1]);
        Vfs = V(1:2);
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        ridgeState= 3;
    elseif Prl(1) < 0 && Prr(1)>0 && R2rleft>= R2Limit && R2rRight < R2Limit
        %Shift right
        MR = [x(ridge_robot_idx)-x(max_robot_idx), y(ridge_robot_idx)-y(max_robot_idx),0];
        V = cross(MR ,[0,0,1]);
        Vfs = V(1:2);
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        ridgeState= 4;
    else
        %Either slopes are not correct or lines fit too poorly
        %Not a ridge so find max
        ridgeState= 2;
        Vfx = 0;
        Vfy = 0; 
    end
end

end