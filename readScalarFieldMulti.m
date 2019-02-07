function [Z]=readScalarField(a,b,ScalarFieldSelection)

% ScalarFieldSelection: 
% 1 : Composite scalar field from Kitts Paper 
% 2 : Single Source 
% 3 : Single Sink 
% % Define scalar field 
xpos=0;
ypos=0;

% scalarFunction = @(x,y) (1).*((x-xpos).^2+(y-ypos).^2).^(.5);
% scalarFunction = @(x,y) .01*log10(((x-xpos).^2+(y-ypos).^2).^(.5))

% scalarFunction = @(x,y) 3*(1-x).^2.*exp(-(x.^2) - (y+1).^2)- 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2)- 1/3*exp(-(x+1).^2 - y.^2); 

% scalarFunction = @(x,y) h*(xs*y+1+yn1)*(xs*x+1)/((d/dhv)^2+1)


% scalarFunction = @(x,y) 10*(1-x).^2.*exp(-(x.^2) - (y+1).^2)- 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2)- 1/3*exp(-(x+1).^2 - y.^2); 


%% Field equation from cluster robot paper

% persistent m1_height m1_rolloff x1_m y1_m

if ScalarFieldSelection == 1
m1_height=50;
m1_rolloff=.0001;
x1_m=215;
y1_m=150;
ScalarFunction= @(x,y) m1_height./(m1_rolloff.*((x-x1_m).^2+(y-y1_m).^2)+1);
% set field width so display makes sense, and set as output of the function

% persistent m2_height m2_rolloff x2_m y2_m

elseif ScalarFieldSelection == 2
m2_height=20;
m2_rolloff=.0001;
x2_m=0;
y2_m=150;
ScalarFunction= @(x,y)  m2_height./(m2_rolloff.*((x-x2_m).^2+(y-y2_m).^2)+1);


% persistent m3_height m3_rolloff x3_m y3_m

elseif ScalarFieldSelection == 3
m3_height=-25;
m3_rolloff=.0001;
x3_m=-100;
y3_m=-150;
ScalarFunction = @(x,y) m3_height./(m3_rolloff.*((x-x3_m).^2+(y-y3_m).^2)+1);

% persistent r1_height r1_er r1_r r1_rx x_r1 y_r1

% r1_height=40;
% r1_er=.005;
% r1_r=.00004;
% r1_rx=-3;
% x_r1=150;
% y_r1=-75;
% R1 = @(x,y) r1_height./(((r1_er.*(y-y_r1)).^4+1).*(r1_r.*(r1_rx.*(x-x_r1)+(y-y_r1)).^2+1));
% 
% % persistent r2_height r2_er r2_ro x_r2 y_r2 x_r2c y_r2c r_r2
% 
% r2_height=25;
% r2_er=.007;
% r2_ro=.001;
% x_r2=-50;
% y_r2=-75;
% x_r2c=-200;
% y_r2c=250;
% r_r2=200;
% R2 = @(x,y) real( r2_height./(((r2_er.*((x-x_r2).^2+(y-y_r2).^2)^(.5)).^4+1).*(r2_ro.*(((x-x_r2c).^2+(y-y_r2c).^2).^(.5)-r_r2).^(2)+1)) );
% 
% % persistent t_height t_er t_r x_t t_x1 t_y1 t_x2 t_y2
% 
% 
% t_height=-15;
% t_er=.01;
% t_r=.03;
% x_t=0;
% t_x1=-100;
% t_y1=-150;
% t_x2=75;
% t_y2=150;
% T = @(x,y) t_height./(((t_er.*(x-x_t)).^4+1).*((t_r.*(abs((t_y2-t_y1).*x-(t_x2-t_x1).*y+t_x2*t_y1-t_y2*t_x1)./((t_x2-t_x1).^2+(t_y2-t_y1).^2).^(0.5))).^2+1));
% % d_t=(abs((t_y2-t_y1).*x-(t_x2-t_x1).*y+t_x2*t_y1-t_y2*t_x1)./(t_dy.^2+t_dx^2).^(0.5));
% 
% 
% scalarFunction= @(x,y) M1(x,y) + M2(x,y)+M3(x,y) + R1(x,y) + R2(x,y) + T(x,y) ;
else 
    disp('No Scalar Field selected, or selection is not defined in readScalarField')
end 

Z = ScalarFunction(a,b);

% % Apply limit to avoid singularity at center of source 
% if z>maxValue
%     z=maxValue;
% end

end