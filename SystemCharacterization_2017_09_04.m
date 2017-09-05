% Data from 2017_09_04_SystemCharacterization
% Trying to see how well our expected torque matches our measured torque
% September 04 2017
%% Experiments
% Trial 1 - Impedance Control (no force feedback) 
% Commanding a torque as a function of the current and 
% Moved the wrist counter clockwise then let go and watched the
% controller stop it. 
% Trial 2 - Trying it again with a stiffer spring 
% Still seems to look good 
% Trial 3 - Now trying to 'surf the cone' here with bang-bang control
% Trial 4 - Matt disturbing while wrist is doing bang-bang
% Trial 5 - Matt disturbed it back and forth, then b/f pulling away, then
% b/f while pushing in 


close all; clear; clc; 

filename = 'data/2017_09_04_SystemCharacterization/01.txt';
filename = 'data/2017_09_04_SystemCharacterization/02.txt';
filename = 'data/2017_09_04_SystemCharacterization/03.txt';
filename = 'data/2017_09_04_SystemCharacterization/04.txt';
filename = 'data/2017_09_04_SystemCharacterization/05.txt';

[stoptorque,goal_current,current,velocity,position,Fx,Fy,Fz,Mx,My,Mz,counterclock,Time] = importTxtData3(filename);


coneFile = 'data/2017_08_28_ConeFollowing/cone.txt';
[Fx1,Fz1,M_max,M_min] = importCone(coneFile);

%% Calculate Moment taken about the motor's axis 
% Adjust by taking out the Fx Component by the moment arm b/t ATI and motor
r_ATI2Motor = 0.05296; % [m]
Madjusted= My+(Fx.*r_ATI2Motor);

N=10; weights=(1/N)*ones(N,1); % Smooth ATI Data a bit 
SmoothMy= filtfilt(weights,1,Madjusted); 

%% Negin' projected current 
P(1)= -0.0056; P(2)= 0.0345;%-0.0210;%-0.113; 
My_Current=(current)*P(1)+sign(velocity).*P(2);

%% Plot 

% 
% ATI Data
plot(Time,Fx,Time,Fz); hold on; 
ylabel('[N]')
yyaxis right
plot(Time,My)
legend('F_x','F_z','M')
title('Raw ATI Data')
xlabel('[ms]')
ylabel('[Nm]')
set(gca,'fontsize',16);

% 
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

% Projected vs. Measured Torque (at Motor axis) 
figure
plot(Time,SmoothMy,'x'); hold on 
plot(Time,Madjusted,'o');
plot(Time,My_Current,'o');
legend('SmoothMy','Madjusted','Currentbased');
xlabel('[ms]')
ylabel('Moment at Motor [Nm]')
set(gca,'fontsize',16);

% Cone following
figure
scatter3(Fx1,Fz1,M_max);
hold on;  
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Measured Moment');
hold on; 
armToMotor = .045;
scatter3(Fx,Fz,Madjusted);
scatter3(Fx,Fz,My_Current);
set(gca,'fontsize',16);
legend('ATI measured','Predicted form Current')

