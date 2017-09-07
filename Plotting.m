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

