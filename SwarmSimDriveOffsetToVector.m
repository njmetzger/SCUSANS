function [Vfx,Vfy,Vt] = SwarmSimDriveOffsetToVector(Vx, Vy, offset)
% SWARMSIMDRIVEOFFSETTOVECTOR - <Determines ......>
%
% Outputs:
%   Rotated velocity vector

%% initialize variables 
% position variables for 3-DOF omnibot are x,y,and theta (rotation about z-axis)
Vf=[0.0 0.0 0.0];
O_ref=0.0;
O_new=0.0;
Mag_ref=1;

%% Algorithm


% Convert reference components to magnitude/angle form
O_ref = atan2( Vy, Vx );
Mag_ref=real(sqrt(Vx^2 + Vy^2));

O_new = O_ref + offset;

% Now recalculate rotated components 
Vfx=double(Mag_ref.*cos(O_new));
Vfy=double(Mag_ref.*sin(O_new));

Vfx=Vfx(1);
Vfy=Vfy(1);
Vt=0;



end