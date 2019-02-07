%circle test - updated criteria for determining "robot right"/"robot left"
clc 
ml_bearing= 3*pi/2;
mlb_flip= ml_bearing+ pi;
mr_bearing= pi;

if mlb_flip >= pi
    mlb_flip= mod(mlb_flip, 2*pi);
else
    mlb_flip=mlb_flip;
end

if ml_bearing >= pi 
    if mr_bearing>0 && mr_bearing < mlb_flip
        disp('robot is to the left') 
    elseif mr_bearing>mlb_flip && mr_bearing < ml_bearing 
        disp('robot is to the right') 
    elseif mr_bearing>ml_bearing && mr_bearing< 2*pi
        disp('robot is to the left') 
    else 
        disp('different case- possibly on master-leader bearing') 
    end 
elseif ml_bearing < pi 
    if mr_bearing>0 && mr_bearing<ml_bearing
        disp('robot is to the right')
    elseif mr_bearing>ml_bearing && mr_bearing <mlb_flip 
        disp('robot is to the left') 
    elseif mr_bearing>mlb_flip && mr_bearing < 2*pi 
        disp('robot is to the right') 
    else 
        disp('different case2-possibly on master_leader bearing')
    end 
end 
    
