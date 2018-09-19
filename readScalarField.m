function [Z]=readScalarField(a,b)

% % Define scalar field 
xpos=-0;
ypos=0;

% scalarFunction = @(x,y) .1.*exp(((x-xpos).^2+(y-ypos).^2).^(.5));
% scalarFunction = @(x,y) .01*log10(((x-xpos).^2+(y-ypos).^2).^(.5))

% scalarFunction = @(x,y) 3*(1-x).^2.*exp(-(x.^2) - (y+1).^2)- 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2)- 1/3*exp(-(x+1).^2 - y.^2); 

% scalarFunction = @(x,y) h*(xs*y+1+yn1)*(xs*x+1)/((d/dhv)^2+1)


scalarFunction = @(x,y) 10*(1-x).^2.*exp(-(x.^2) - (y+1).^2)- 10*(x/5 - x.^3 - y.^5).*exp(-x.^2-y.^2)- 1/3*exp(-(x+1).^2 - y.^2); 



Z = scalarFunction(a,b);

% % Apply limit to avoid singularity at center of source 
% if z>maxValue
%     z=maxValue;
% end

end