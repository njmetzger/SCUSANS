function [Vf,trenchState]= SwarmSimFollowTrench(RobotParams, NRobot,SensorRange)
% SWARMSIMFOLLOWRIDGE - <Determines....>

% Outputs
%   trenchState      1=OnTrench, 2= offTrench, 3= OffTrench_Right
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
O_minToRobots = zeros(1,N);
d_from_min=zeros(1,N);
delta_z_from_min=zeros(1,N);
amp=zeros(1,N);
% ridgeVector=[0.0 0.0 0.0];
Vfx=0.0;
Vfy=0.0;
Vft=0.0;
Vf = [0.0 0.0 0.0];
trenchState = 0;



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
if length(inRange_idx) >=6
    min_robot_idx=find(SensorValue==min(SensorValue(inRange_idx)));
    if length(min_robot_idx)>1
        min_robot_idx = min_robot_idx(1);
    end
    TrenchBuffer = 0.1*(max(SensorValue(inRange_idx)-min(SensorValue(inRange_idx))));
    
    d_from_min = sqrt((x(min_robot_idx)*ones(size(x))-x ).^2 + ( y(min_robot_idx)*ones(size(x))-y).^2 );
    delta_z_from_min = SensorValue-SensorValue(min_robot_idx)*ones(size(SensorValue));
    amp= (d_from_min.^2)./delta_z_from_min;
    amp(delta_z_from_min<TrenchBuffer)=0;
    % Find max "amplitude" robot
    trench_robot_idxs=find(amp>=0.9*max(amp(inRange_idx)));
    T2rRight_b = 0;
    T2rLeft_b = 0;
    
    for n = 1:length(trench_robot_idxs)
        trench_robot_idx = trench_robot_idxs(n);
        [Vfx_t,Vfy_t,trenchState_t,T2rleft,T2rRight]= checktrench(min_robot_idx,trench_robot_idx,x,y,SensorValue,N,NRobot,inRange_idx);
        if T2rleft>T2rLeft_b && T2rRight>T2rRight_b && trenchState_t ==1
            T2rRight_b = T2rleft;
            T2rLeft_b =T2rleft ;
            Vfx= Vfx_t;
            Vfy=Vfy_t;
            trenchState= trenchState_t;
        elseif trenchState ==0
            trenchState = trenchState_t;
            Vfx= Vfx_t;
            Vfy=Vfy_t;
        end
        
    end
    
    
else
    %Not enough robots inrange
    trenchState= 5;
    min_robot_idx = find(SensorValue== min(SensorValue(inRange_idx)));
    if length(min_robot_idx)>1
        min_robot_idx = min_robot_idx(1);
    end
    switch NRobot
        case min_robot_idx
            Vfx=0;
            Vfy=0;
        otherwise
            Vx=x(min_robot_idx)-x(NRobot);
            Vy=y(min_robot_idx)-y(NRobot);
            Vfx = Vx/sqrt(sum(Vx.^2+Vy.^2));
            Vfy = Vy/sqrt(sum(Vx.^2+Vy.^2));
    end
    
end
Vf = [Vfx(1) Vfy(1) Vft(1)];
end
%% Helper Functions
function [Vfx,Vfy,trenchState,T2rleft,T2rRight]= checktrench(min_robot_idx,trench_robot_idx,x,y,SensorValue,N,NRobot,inRange_idx)
%Compute Homogeneous transform to look along vector from trenchRobot
%to min robot to determine number and curvature of robots on either
%side of proposed trench

%Find angles theta1 and theta2 for transform

theta1 = atan2(y(trench_robot_idx) - y(min_robot_idx),x(trench_robot_idx) - x(min_robot_idx));
theta2 = atan2(SensorValue(min_robot_idx)-SensorValue(trench_robot_idx),sqrt((y(trench_robot_idx) - y(min_robot_idx)).^2+(x(trench_robot_idx) - x(min_robot_idx))^2));
%Build Transforms
R1 = [cos(theta1), sin(theta1),0;-sin(theta1),cos(theta1),0;0 0 1 ;];
P01 = [x(min_robot_idx);y(min_robot_idx); SensorValue(min_robot_idx)];
H1 = [R1 -R1*P01; 0 0 0  1];
R2 = [cos(theta2),0, -sin(theta2);0 1 0; sin(theta2),0,cos(theta2)];
H2 = [R2 [0;0;0]; 0 0 0 1];
H = H2*H1;
%Convert coordinates
TPa = zeros(N*4,1);
TPa(1:4:end)=x;
TPa(2:4:end)=y;
TPa(3:4:end)= SensorValue;
TPa(4:4:end) = 1;
ACell = repmat({H}, 1, N);
HM = blkdiag(ACell{:});
TPap = HM*TPa;
Xsp = TPap(1:4:end);
Ysp = TPap(2:4:end);
SVp = TPap(3:4:end);
Xsp = Xsp(inRange_idx);
Ysp = Ysp(inRange_idx);
SVp = SVp(inRange_idx);


%Determine if robots are on a trench
nr_right = sum(Ysp<0);
nr_left = sum(floor(Ysp)>0);
if nr_right>0
    [Prr,Srr] = polyfit(Ysp(Ysp<=0 & Xsp>=0),SVp(Ysp<=0 & Xsp>=0),1);
    T2rRight = 1 - (Srr.normr/norm(SVp(Ysp<=0 & Xsp>=0) - mean(SVp(Ysp<=0 & Xsp>=0))))^2;
else
    Prr = 0;
    T2rRight =0;
end
if nr_left>0
    [Prl,Srl] = polyfit(Ysp(Ysp>=0 & Xsp>=0),SVp(Ysp>=0 & Xsp>=0),1);
    T2rleft = 1 - (Srl.normr/norm(SVp(Ysp>=0 & Xsp>=0) - mean(SVp(Ysp>=0 & Xsp>=0))))^2;
else
    Prl = 0;
    T2rleft = 0;
end
N_sided = floor(length(inRange_idx)/4);
if nr_right >0 && nr_left >0
    if Prl(1) > 0 && Prr(1)<0 && T2rleft>0.7 && T2rRight > 0.7 &&  nr_right >=N_sided && nr_left >=N_sided
        %Ridge with sufficeint number of robots on either side
        Vfs = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx)];
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        trenchState=1;
        
        %Ridge found but but sideslip needed
    elseif nr_right>N_sided
        MR = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx),0];
        V = cross(MR ,[0,0,-1]);
        Vfs = V(1:2);
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        trenchState= 3;
    elseif nr_left> N_sided
        MR = [x(trench_robot_idx)-x(min_robot_idx), y(trench_robot_idx)-y(min_robot_idx),0];
        V = cross(MR ,[0,0,1]);
        Vfs = V(1:2);
        Vm = sqrt(sum(Vfs.^2));
        Vfx=Vfs(1)/Vm;
        Vfy=Vfs(2)/Vm;
        trenchState= 4;
    else
        %Not a ridge
        trenchState= 2;
        switch NRobot
            case min_robot_idx
                Vfx=0;
                Vfy=0;
            otherwise
                Vx=x(min_robot_idx)-x(NRobot);
                Vy=y(min_robot_idx)-y(NRobot);
                Vfx = Vx/sqrt(sum(Vx^2+Vy^2));
                Vfy = Vy/sqrt(sum(Vx^2+Vy^2));
        end
    end
else
    %Not a trench
    trenchState= 2;
    switch NRobot
        case min_robot_idx
            Vfx=0;
            Vfy=0;
        otherwise
            Vx=x(min_robot_idx)-x(NRobot);
            Vy=y(min_robot_idx)-y(NRobot);
            Vfx = Vx/sqrt(sum(Vx^2+Vy^2));
            Vfy = Vy/sqrt(sum(Vx^2+Vy^2));
    end
    
end
end