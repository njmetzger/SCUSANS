N=5; 

legend_labels=cell(1,(N+1)); 

for i= 1:N
    leg_str1= 'Robot Number  '
    rob_num_legend= num2str(i)
    legend_labels{1,i} = strcat(leg_str1, rob_num_legend);
    %plot(time, sensor_value_PI(:,i)) 
%     title('Sensor Value Readings')
end 

legend_labels{1,N+1}= 'Desired Contour'; 


t= linspace(0,10,100); 
x1= t; 
x2= 2*t; 
x3= 3*t; 
x4= 4*t; 
x5= 5*t; 
x6= -2.5*t; 

hold on 
plot (t,x1); 
plot (t,x2); 
plot (t,x3); 
plot (t,x4); 
plot(t,x5); 
plot(t,x6); 
title('test plots') 
legend(legend_labels)