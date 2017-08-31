% Verifying the relationship between torque commanded at the motor and
% force sensed at the ATI
% August 30, 2017 
% Matt Estrada

function [ tau_Fx, tau_M, tau_cur] = StaticVerification(filename,PLOT)

%filename = 'data/2017_08_30_StaticVerification/190.txt';
[stoptorque,goal_current,current,velocity1,position,Fx1,Fy,Fz1,Mx,My,Mz,counterclock,Time1] = importTxtData3(filename);

tau_Fx = median(Fx1); 
tau_M = median(My);
tau_cur = median(current); 

if(PLOT)
    figure
    % ATI
    plot(Time1,Fx1,Time1,Fz1); hold on; 
    ylabel('[N]')
    yyaxis right
    plot(Time1,My)
    legend('F_x','F_z','M')
    title('Raw ATI Data')
    xlabel('[ms]')
    ylabel('[Nm]')
    set(gca,'fontsize',16);

    % Motor Current 
    figure
    subplot(2,1,1)
    plot(Time1,goal_current,'k*',Time1,current)
    xlabel('[ms]')
    ylabel('Amps???')
    legend('Goal','Actual')
    title('Motor Current') 
    set(gca,'fontsize',16);

    % Velocity 
    subplot(2,1,2)
    plot(Time1,velocity1)
    ylabel('[Deg/sec]')
    xlabel('[ms]')
    title('Joint Velocity')
    set(gca,'fontsize',16);

    % Velocity 
    subplot(2,1,2)
    plot(Time1,velocity1)
    ylabel('[Deg/sec]')
    xlabel('[ms]')
    title('Joint Velocity')
    set(gca,'fontsize',16);

    figure 
    plot(current,Fx1,'r.'); hold on; 
    ylabel('[N]')
    yyaxis right
    plot(current,My,'b.')
    plot(tau_cur,tau_M,'bs','MarkerSize',20,'LineWidth',3); 
    legend('F_x','M')
    xlabel('[ms]')
    ylabel('[Nm]')
    yyaxis left
    plot(tau_cur,tau_Fx,'rs','MarkerSize',20,'LineWidth',3)
    set(gca,'fontsize',16);
end
