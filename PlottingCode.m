% 722 pull rotate push rotate the bump in the table might be affecting the
% data also it hit a hardstop on one side
% 740 moved the cylinder so less bumps but still hit a few bumps
% 744 more carefull not to hit bumps 
% 821 I tried to rotate it with my hand a bit sketchy since the magnets
% were close to detaching when a torque is being applied 
%1220 was with the passive Compliant in the back


% September 5 
% Throwing the object on white board table 
%821 with the cone limit 
%838 cone limit changed


%% Distinguish when the Bang-Bang control was active/ in-active 
inactive=[];
currentinactive=[];
Finactive=[];
active=[];
Fxactive=[];
currentactive=[];
Myactive=[];
Fzactive=[];
Timeactive=[];
velocityactive=[];
goal_currentactive=[];

for i=1:size(Fx)
    if(1>2)%(abs(velocity(i))<4)
        inactive=[inactive i];
        Finactive=[Finactive Fx(i)];
        currentinactive=[currentinactive current(i)];
        
        
    else 
        active=[active i];
        Fxactive=[Fxactive Fx(i)];
        currentactive=[currentactive current(i)];
        goal_currentactive=[goal_currentactive goal_current(i)];
        Myactive=[Myactive My(i)];
        Fzactive=[Fzactive Fz(i)];
        Timeactive=[Timeactive Time(i)];
        velocityactive=[velocityactive velocity(i)];
        
    end 
end 



%%
arm= 0.05296;
Madjusted= Myactive-(Fxactive.*arm);
N=10;
weights=(1/N)*ones(N,1);
SmoothMy= filtfilt(weights,1,Madjusted);
plot(Timeactive,SmoothMy,'x');
hold on 
plot(Timeactive,Madjusted,'.');

P(1)=  -0.0026;
P(2)= 0.0004149%0.0313; %0.0345;%-0.0210;%-0.113;
test=(currentactive)*P(1)+P(2)*sign(velocityactive);
N=10;
weights=(1/N)*ones(N,1);
Smoothtest= filtfilt(weights,1,test);
plot(Timeactive,test,'o');
test2=(goal_currentactive)*P(1)+P(2)*sign(velocityactive);
N=10;
weights=(1/N)*ones(N,1);
Smoothtest= filtfilt(weights,1,test);
hold on
plot(Timeactive,test2,'x');
xlabel('Time (ms)');
ylabel('Nm ');
legend('SmoothMy','Madjusted','Currentbased','goal_current based');
stepsize=[];
for i=2:size(Time)
    stepsize(i-1,1)= Time(i,1)-Time(i-1,1);
end 
%%
N=10;
weights=(1/N)*ones(N,1);
Smoothv= filtfilt(weights,1,velocity);
a=diff(Smoothv)./stepsize;
figure
plot(Time(1:end-1),a,'x');
%%
figure;
yyaxis right
plot(Time(1:end-1),a,'x');
yyaxis left
plot(Timeactive,test,'y.',Timeactive,SmoothMy,'bx');

%% Plot the cone 
figure
scatter3(Fx1,Fz1,M_max);
hold on
scatter3(Fx1,Fz1,M_min);
xlabel('Fx(N)');
ylabel('Fz(N)');
zlabel('M(Nm)');
title('Cone Generate through Visual Studio');

test2=(goal_currentactive)*P(1)+sign(velocityactive).*P(2);
scatter3(Fxactive,Fzactive,SmoothMy);
hold on
scatter3(Fxactive,Fzactive,test);
hold on 
scatter3(Fxactive,Fzactive,test2);
legend('cone','cone','SmoothMy','based on current','based on goal_current');
%%
figure 
plot(Time,Fx);
hold on
plot(Time,Fz);
plot(Time,SmoothMy);
legend('Fx','Fz','My');

%%
for i=1:(size(Time)-1)
    if(a(i,1)>0.01 || a(i,1)<-0.01 )
    plot(Time(i,1),(test(1,i)-SmoothMy(1,i))./a(i,1),'x');
    hold on
    end
end 







