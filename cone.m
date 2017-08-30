% Aug 29 2017 
% Negin and Matt looking at data from first experiments commanding a
% "cone" contstraint 

close all; clear; clc;

%% Import Data 
dataFile = 'data/2017_08_28_ConeFollowing/Plots.txt';
coneFile = 'data/2017_08_28_ConeFollowing/cone.txt';

[stoptorque,goal_current,current,velocity,position,Fx,Fy,Fz,Mx,My,Mz,counterclock,Time]=...
importData(dataFile);
[Fx1,Fz1,M_max,M_min] = importCone(coneFile);

% Plotting Desired Moment 
scatter3(Fx1,Fz1,M_max); hold on;
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Theoretical Commanded Motor Torque');
sgcurrent=counterclock.*current;
calculatedTorque=sgcurrent*2.69/1000
calculatedTorque=calculatedTorque*1.0709;
scatter3(Fx,Fz,calculatedTorque);
set(gca,'fontsize',16);

%scatter3(Fx,Fz,counterclock.*newMy); % ??? what was this? 

% Plotting Actual measured torque 
figure;
scatter3(Fx1,Fz1,M_max);
hold on;  
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Measured Moment');
hold on; 
armToMotor = .045;
MyAdjusted = My + Fx.*armToMotor; 
scatter3(Fx,Fz,My);
set(gca,'fontsize',16);

% Commanded vs. measured torque 
figure 
plot(Time,-calculatedTorque,Time,My)
xlabel('[ms]');
ylabel('[Nm])');
title('Commanded vs. Measured Moment');
legend('Commanded','Measured')
set(gca,'fontsize',16);

% Commanded vs. measured torque 
% NOT RELEVANT YET 
figure 
plot(current,My,'.',current,calculatedTorque,'.')
xlabel('Commanded current [A?]');
ylabel('Measured Moment [Nm]');
set(gca,'fontsize',16);
legend('Measured','Calculated')


figure
plot(Time,velocity.*counterclock)
xlabel('[ms]');
ylabel('[??])');

