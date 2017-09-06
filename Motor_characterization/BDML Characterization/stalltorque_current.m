figure; 
hold on;
Arm=0.056;
current=[current; current1];
My=[My; My1];
Fx=[Fx; Fx1];
plot(current,My+(Arm*Fx),'x');
catcurrent=[];
catMy=[];
catFx=[];
%%
for i= 1:size(current,1)
    if (current(i,1)>30)
        catcurrent=[catcurrent current(i,1)];
        catMy=[catMy My(i,1)];
        catFx=[catFx Fx(i,1)];
        
    end 
end
%%

figure
scatter(catcurrent,catMy+(Arm*catFx),25,'b','*')
P = polyfit(catcurrent,catMy+(Arm*catFx),1);
yfit = P(1)*catcurrent+P(2);
hold on;
plot(catcurrent,yfit,'r-.');
%P was found to be -0.0023 0.0313 
%%
figure 
scatter(catcurrent*(2.69/1000),-(catMy+(Arm*catFx)),25,'b','*')
P = polyfit(catcurrent*(2.69/1000),-(catMy+(Arm*catFx)),1);
yfit = P(1)*catcurrent*(2.69/1000)+P(2);
hold on;
plot(catcurrent*(2.69/1000),yfit,'r-.');
xlabel('Current (A)')
ylabel('Torque (Nm)')
% P was found to be 0.8714 -0.0313 







