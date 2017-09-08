% Second day doing test catches
% Sept 7 2017
% More throws on the whiteboard table in BDML 
% Trial 1 - active bang-bang with thresh hold of 20 [dynamixel units] on
% velocity for activation
% Trial 2 - same with threshholdof 4
% Both these trials started capturing data once velocity thresh hold hit so Negin changed that 
% 
% Trial 3 - Spinning clockwise 
% Negin changed the cone 
% partial grasp -- it grasped 1 or 2 cm out from the gripper 
% Trial 4 - 

close all; clear; clc; 

filename = 'Trial4.txt';
coneFile = 'ConeV2.txt';

[stoptorque,goal_current,current,velocity,position,Fx,Fy,Fz,Mx,My,Mz,counterclock,Time] = importTxtData3(filename);
[Fx1,Fz1,M_max,M_min] = importCone(coneFile);


%% Calculate the projected Moment at the motor from the current 
arm= 0.05296;
P(1)=  0.0026;
P(2)= 0;

Madjusted= My-(Fx.*arm);

%% Raw ATI Data

% ATI Data
figure
plot(Time,Fx,Time,Fz); hold on; 
ylabel('[N]')
yyaxis right
plot(Time,My)
legend('F_x','F_z','M')
title('Raw ATI Data')
xlabel('[ms]')
ylabel('[Nm]')
set(gca,'fontsize',16);
%% Current, Velocity, Position 

figure
subplot(3,1,1)
plot(Time,goal_current,'ko',Time,current,'r')
xlabel('[ms]')
ylabel('Current [??]')
legend('Goal','Actual')
title('Motor Current') 
set(gca,'fontsize',16);

subplot(3,1,2)
plot(Time,velocity)
ylabel('[Deg/sec]')
xlabel('[ms]')
title('Joint Velocity')
set(gca,'fontsize',16);

subplot(3,1,3)
plot(Time,position)
ylabel('[Deg??]')
xlabel('[ms]')
title('Joint Position')
set(gca,'fontsize',16);

%% Current 
Mcurrent=(current)*P(1)+P(2)*sign(velocity);
Mgoal=(goal_current)*P(1)+P(2)*sign(velocity);

figure
plot(Time,Madjusted,'-','LineWidth',3); hold on
plot(Time,-Mcurrent,'-','LineWidth',1);
plot(Time,-Mgoal,'o');
xlabel('Time (ms)');
ylabel('Nm ');
legend('Madjusted','Current based','goal current based');

 %% Plot the cone 
figure
scatter3(Fx1,Fz1,M_max);
hold on
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Cone Generate through Visual Studio');
scatter3(Fx,Fz,Mgoal); %based on goal_current
hold on 
scatter3(Fx,Fz,Mcurrent); %based on Smooth current 
hold on 
scatter3(Fx,Fz,Madjusted);% based on M ATI 
legend('cone','cone','goal_current','smooth current','M ATI');

