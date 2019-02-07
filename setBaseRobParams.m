% script to put the info from base_RobotData into usable RobotParams for
% testing trench following function
clc
clear x y SV theta
for i=1:5
    x(:,i)= base_RobotData(i).x;
    y(:,i)= base_RobotData(i).y ;
    SV(:,i)= base_RobotData(i).sensor_value; 
    theta(:,i)= base_RobotData(i).theta; 
end 
x=x' ;
y= y';
SV= SV' ;
theta= theta';
for i= 1:5
    for j= 1: 100 
        RobotParams(i,4*j-3)= x(i,j);
        RobotParams(i,4*j-2)= y(i,j); 
        RobotParams(i,4*j-1)= SV(i,j) ;
        RobotParams(i, 4*j) = theta(i,j); 
    end 
end 

%% Determine different (x,y,z) that are on ridge, offset left and right, and then test the logic here 
x= [0 4 1 5]; 
y= [0 0 5 5]; 
SV= [10 6 4.5 0]; 

for i= 1: length(x); 
    for j= 1: length(x); 
        if i==j 
            d(i,j) = NaN; 
        else 
            d(i,j) = sqrt((x(i)-x(j))^2 + (y(i)-y(j))^2); 
            O(i,j) = atan2((y(j)-y(i)), (x(j)-x(i))); 
        end 
    end 
end 

for i= 1:length(x); 
    for j=1:length(x); 
        if i==j 
            local_grad(i,j)= NaN; 
        else 
            local_grad(i,j)= (SV(i)-SV(j))/d(i,j); 
        end 
    end 
end 

[f_row, f_col]= find(local_grad == min(min(abs(local_grad))))

if local_grad(f_row,f_col)> 0 
    master_robot_idx= f_row
    leader_robot_idx= f_col
elseif local_grad(f_row,f_col)<0  
    master_robot_idx= f_col 
    leader_robot_idx= f_row 
else 
    disp('2 robots have the same sensor value readings') 
end 

if SV(f_row) > SV(f_col) 
    master_robot_index= f_row
    leader_robot_index= f_col
elseif SV(f_row) < SV(f_col) 
    master_robot_index= f_col
    leader_robot_index= f_row
else 
    disp('same sensor value') 
end 
master_leader_bearing= O(master_robot_index, leader_robot_index);  
num_robots_left= 0; 
num_robots_right= 0; 
for i= 1:length(x)
    if i ~= master_robot_index && i~=leader_robot_index
        if O(master_robot_index, i) > master_leader_bearing 
            num_robots_left= num_robots_left+1; 
        elseif O(master_robot_index,i) < master_leader_bearing 
            num_robots_right= num_robots_right+1; 
        end 
    end 
end 

% Determine Swarm State 

sensor_val_mast_flag= 0; 
roles_filled_flag = 0; 

%check if the sensor value of the "master" is the highest of the local
%group and change flag to 1 if true 
if SV(master_robot_index) == max(SV) 
    sensor_val_mast_flag = 1; 
else 
    sensor_val_mast_flag= 0; 
end 

%check if all roles filled and change the flag to 1 if true 
if num_robots_left>0 && num_robots_right>0 
    roles_filled_flag = 1; 
else 
    roles_filled_flag = 0; 
end 

if sensor_val_mast_flag && roles_filled_flag 
    disp('Swarm is on a ridge') 
elseif roles_filled_flag && ~sensor_val_mast_flag
    disp('Swarm is not on a ridge') 
elseif sensor_val_mast_flag && ~roles_filled_flag
    if num_robots_left==0 
        disp('swarm is offset to the right') 
    elseif num_robots_right==0 
        disp('swarm is offset to the left') 
    end 
else 
    disp('logic failed') 
end 


    


