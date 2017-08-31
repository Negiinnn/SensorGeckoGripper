% Verifying the relationship between torque commanded at the motor and
% force sensed at the ATI from Negin experiments
% August 30, 2017 
% Matt Estrada
close all; clear; clc; 

files = {...
    'data/2017_08_30_StaticVerification/20.txt';...
    'data/2017_08_30_StaticVerification/30.txt';...
    'data/2017_08_30_StaticVerification/40.txt';...
    'data/2017_08_30_StaticVerification/80.txt';...
    'data/2017_08_30_StaticVerification/100.txt';...
    'data/2017_08_30_StaticVerification/170.txt';...
    'data/2017_08_30_StaticVerification/190.txt'}

n = length(files);
tau_cur = zeros(1,n); 
tau_Fx = zeros(1,n); 
tau_M = zeros(1,n); 

for filenum = 1:n 
    filenum
    [this_Fx, this_M, this_cur] = StaticVerification(files{filenum},0);
    tau_Fx(filenum) = this_Fx;
    tau_M(filenum) = this_M;  
    tau_cur(filenum) = this_cur*2.69;  
end

plot(tau_cur,tau_Fx,'o','Markersize',8); hold on; 
xlabel('Current [mA]')
ylabel('Measured ATI Force')
set(gca,'fontsize',16);
yyaxis right
plot(tau_cur,tau_M,'o','Markersize',8)
ylabel('Measured ATI Moment')
Legend('Horizontal Force','Moment at ATI')


%%
StaticVerification('data/2017_08_30_StaticVerification/190.txt',1);
