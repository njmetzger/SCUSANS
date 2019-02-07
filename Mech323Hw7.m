%% MECH 323 HW 7 

% confirm process done for problems 1 and 2 with the textbook examples,
% re-write 1 into hard copy paper. 

%% Problem 1, part i 
clear 
clc 

A= [0 1; -2 -3]; 
B= [ 0; 1]; 
C= [ 1 1]; 
D= 0; 

sys1= ss(A,B,C,D)

[num1, den1]= ss2tf(A,B,C,D)
tf1= tf(num1,den1)
zpk1= zpk(tf1)

minreal1= minreal(zpk1)
[num1_mr, den1_mr]= tfdata(minreal1,'v')

[A_mr, B_mr, C_mr, D_mr] = tf2ss(num1_mr,den1_mr)

%% Problem 1, Part ii 
clear 
clc

A= [ 0 1 0; 
    0 0 1; 
    -15, -17, -7]; 
B= [0; 0; 1]; 
C= [5 4 1]; 
D= 0; 


sys1= ss(A,B,C,D);
[num1, den1]= ss2tf(A,B,C,D)
tf1= tf(num1,den1);
zpk1= zpk(tf1)
 
 minreal1= minreal(zpk1);
[num1_mr, den1_mr]= tfdata(minreal1,'v');

[A_mr, B_mr, C_mr, D_mr] = tf2ss(num1_mr,den1_mr)

%% Problem 2 

clear 
clc

A= [ 0 1 0 0; 
    0 0  1 0; 
    0 0 0  1; 
    -250, -255, -91, -15]; 
B= [0; 0; 0; 1];
C= [25 8 1 0]; 
D= 0;
    
% check if reducible: 

ctrb1= ctrb(A,B);
ctrb1_rank= rank(ctrb1)
obsv1= obsv(A,C); 
obsv1_rank= rank(obsv1);

sys1= ss(A,B,C,D);
[num1, den1]= ss2tf(A,B,C,D);
tf1= tf(num1,den1);
zpk1= zpk(tf1)
 
 minreal1= minreal(zpk1)
[num1_mr, den1_mr]= tfdata(minreal1,'v')

[A_mr, B_mr, C_mr, D_mr] = tf2ss(num1_mr,den1_mr)
% [A_mr1, B_mr1, C_mr1, D_mr1]= minreal(A,B,C,D)
