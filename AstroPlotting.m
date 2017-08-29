clc; clear; close all; 

[GoalCurrent,Current,velocityTicks,Position,Torque,Fx,Fz,Time] = importTxtData('Plots.txt');
velocity = velocityTicks * .229 * 360 / 60; % rpm * 360/60 = [deg/sec]

figure
% ATI
plot(Time,Fx,Time,Fz); hold on; 
ylabel('[N]')
yyaxis right
plot(Time,Torque)
legend('F_x','F_z','M')
title('Raw ATI Data')
xlabel('[ms]')
ylabel('[Nm]')
set(gca,'fontsize',16);

% Motor Current 
figure
subplot(2,1,1)
plot(Time,GoalCurrent,'k*',Time,Current)
xlabel('[ms]')
ylabel('Amps???')
legend('Goal','Actual')
title('Motor Current') 
set(gca,'fontsize',16);

% Velocity 
subplot(2,1,2)
plot(Time,velocity)
ylabel('[Deg/sec]')
xlabel('[ms]')
title('Joint Velocity')
set(gca,'fontsize',16);





