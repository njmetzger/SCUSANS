
close all

% First plot scalar field 
figure(1)
ax=gca;
ax.XLim=[-3 3];
ax.YLim=[-3 3];
res=100;
xdivs=linspace(ax.XLim(1),ax.XLim(2),res);
ydivs=linspace(ax.YLim(1),ax.YLim(2),res);
[X,Y] = meshgrid(xdivs,ydivs);
Z=readScalarField(X,Y);


surf(X,Y,Z);
hold on
view([0 90])
axis([-3 3 -3 3])

zindex=4;

DesiredValue=2;

[M,c] =contour3(X,Y,Z,[DesiredValue DesiredValue],'ShowText','on')
c.LineWidth = 3;

figure(2)
d=abs(Z-DesiredValue);
r=atan(d);
Z2=exp(7*r);
surf(X,Y,Z2);