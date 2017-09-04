% Data from 2017_09_04_SystemCharacterization
% Trying to see how well our expected torque matches our measured torque
% September 04 2017
close all; clear; clc; 

filename = 'data/2017_09_04_SystemCharacterization/01.txt';
[stoptorque,goal_current,current,velocity,position,Fx,Fy,Fz,Mx,My,Mz,counterclock,Time] = importTxtData3(filename);

% Moment taken about the motor's axis 
% Adjust by taking out the Fx Component by the moment arm b/t ATI and motor
r_ATI2Motor = 0.05296; % [m]
Madjusted= My+(Fx.*r_ATI2Motor);

N=10; weights=(1/N)*ones(N,1); % Smooth ATI Data a bit 
SmoothMy= filtfilt(weights,1,Madjusted); 

plot(Time,SmoothMy,'x'); hold on 
plot(Time,Madjusted,'o');
P(1)= -0.0056; P(2)= 0.0345;%-0.0210;%-0.113;

test=(current)*P(1)+sign(velocity).*P(2);
plot(Time,test,'o');
legend('SmoothMy','Madjusted','Currentbased');
xlabel('[ms]')
ylabel('My Moment measured/projected at ATI [Nm]')
    set(gca,'fontsize',16);
