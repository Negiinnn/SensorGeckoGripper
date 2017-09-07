% Second day doing test catches
% Sept 7 2017
% Active bang
% Trial 1 - active bang-bang with thresh hold of 20 [dynamixel units] on
% velocity for activation
% Trial 2 - same with threshholdof 4

close all; clear; clc; 

filename = 'Trial1.txt';
[stoptorque,goal_current,current,velocity,position,Fx,Fy,Fz,Mx,My,Mz,counterclock,Time] = importTxtData3(filename);
coneFile = 'ConeV2.txt';
[Fx1,Fz1,M_max,M_min] = importCone(coneFile);

%%
arm= 0.05296;
P(1)=  0.0026;
P(2)= 0;

Madjusted= My-(Fx.*arm);

plot(Time,Madjusted,'.');
hold on


test=(current)*P(1)+P(2)*sign(velocity);
plot(Time,-test,'o');
test2=(goal_current)*P(1)+P(2)*sign(velocity);
plot(Time,-test2,'o');
xlabel('Time (ms)');
ylabel('Nm ');
legend('Madjusted','Currentbased','goal_current based');

%%
 %% Plot the cone 
figure
scatter3(Fx1,Fz1,M_max);
hold on
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Cone Generate through Visual Studio');
scatter3(Fx,Fz,test2); %based on goal_current
hold on 
scatter3(Fx,Fz,test); %based on Smooth current 
hold on 
scatter3(Fx,Fz,Madjusted);% based on M ATI 
legend('cone','cone','goal_current','smooth current','M ATI');

