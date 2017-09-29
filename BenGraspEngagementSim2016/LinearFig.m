
clear all
clc

hold on

load('AllLinearTests Landing Envelope 15-Sep-2015.mat')

phi = 2;
vmag = 1;
omega = 3;
offset = 4;

phi_sweep = (-5:1:90)*pi/180;
omega_limit = 150*pi/180;
m_april = 1.55; % kg
I_april = 0.0128; % kgm^2
rgy = sqrt(I_april/m_april); 
r = 9*0.0254/2+.01; 
v = omega_limit*(rgy^2+r^2)./(r*sin(phi_sweep));



scatter(envelope.success(:,phi),envelope.success(:,vmag),40,[0,.7,0],'o','LineWidth',2)
scatter(envelope.failure(:,phi),envelope.failure(:,vmag),90,'r','x','LineWidth',2);

vnormal = 0.1./cos(phi_sweep);

legend('success', 'failure');


h4 = patch([phi_sweep(7:end)*180/pi,100], [v(7:end),2], [1,1,1],'EdgeColor',[1,1,1]);
h5 = patch([phi_sweep*180/pi,100,100,-100], [vnormal,2,-1,-1], [1,1,1],'EdgeColor',[1,1,1]);

h1 = patch([phi_sweep(7:end)*180/pi,100], [v(7:end),2], [1,.095,.095],'EdgeColor',[1,0,0],'Facealpha',.07);
h2 = patch([phi_sweep*180/pi,100,100,-100], [vnormal,2,-1,-1], [1,.095,.095],'EdgeColor',[1,0,0],'Facealpha',.07);


h3 = patch([-100,100,100,-100],[-1,-1,2,2],[.95,1,.95],'EdgeColor',[0,.7,0]);


xlabel('Angle of Attack \phi [deg]','FontSize',16)
ylabel('Velocity Magnitude [m/s]','FontSize',16)
axis([-5 90 0 1])

uistack(h2,'bottom');
uistack(h1,'bottom');
uistack(h4,'bottom');
uistack(h5,'bottom');
uistack(h3,'bottom');