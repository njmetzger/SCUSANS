function [ThetaCommand] = SwarmSimSum(Attract, Disperse, Avoidance, FindMin, FindMax)
%SwarmSimSum Aggregates the velocity inputs from individual behaviors into a single heading command. 
%   SwarmSimSum is the function called in the Sum blocks in
%   Swarm_Adaptive_Navigation_Simulator/Robot # Behavior. Each behavior
%   outputs a Vf vector with components [Vfx, Vfy, Vft]. This function
%   calculates the sum of all of the behaviors and outputs a theta value
%   using atan2 that is then fed to the robot level. 

% TO ADD A BEHAVIOR TO THE SUM FUNCTION: 
% 1) Add the behavior to the inputs for the SwarmSimSum function
% 2) Add the behavior to x_agg, y_agg, and t_agg sums below. 

% Initialize Variables
x_agg= 0; 
y_agg= 0; 
ThetaCommand = 0; 
% Sum the x-components of the input velocity vectors 

x_agg = Attract(1) + Disperse(1) + Avoidance(1) + FindMin(1) + FindMax(1); % + NewBehavior(1) % 

% Sum the y-components of the input velocity vectors 

y_agg = Attract(2) + Disperse(2) + Avoidance(2) + FindMin(2) + FindMax(2); % + NewBehavior(2) %  

% Sum the theta-components of the input velocity vectors 

% t_agg = Attract(3) + Disperse(3) + Avoidance(3) + FindMin(3) + FindMax(3) % + NewBehavior(3) %  


ThetaCommand = atan2(y_agg, x_agg); 

end

