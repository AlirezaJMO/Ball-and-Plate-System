%% Open Simulink

open MotorX_PlantX
open MotorXY_Plant

%%
[A,B,C,D] = linmodv5('MotorX_PlantX');
rank(ctrb(A,B)); %Controllability check
rank(obsv(A,C)); %Observability check
sys0 = ss(A,B,C,D) %Conversion to state-space model
sys0 = tf(sys0) %Conversion to transfer function
Ts = 0.01; %Desired Sample time
[A_D,B_D] = c2d(A,B,Ts) %Conversin from Continuous-time to Discrete-time
[numz,denz] = ss2tf(A_D,B_D,C,D); %Conversion to Discrete transfer function
Gz = tf(numz,denz,Ts) %Discrete transfer function of system
  
%%
x_initial = [0.1, 0]; %Intial value of ball on the plate
p = [exp(-60*Ts), exp(-60*Ts)]; %Desired pole values for system
K = acker(A_D,B_D,p) %Pole placemet using acker function and obtaining the gain values for state-feedback
A_bar = A_D-B_D*K; % the feedback to the system
sys1 = ss(A_bar,B_D,C,D,Ts);
Gc = tf(sys1); 
[y,tOut] = initial(sys1,x_initial,0:Ts:0.2);
plot(tOut,y) %Plotting the response of system to initial value of ball