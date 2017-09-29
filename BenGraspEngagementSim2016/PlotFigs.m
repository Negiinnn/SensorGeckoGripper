hold on

%Plot simulated data (mirrored)
% scatter([angvel_success,-angvel_success],[offset_success,-offset_success]*100,1,[0,.7,0],'filled');
% scatter([angvel_failure,-angvel_failure],[offset_failure,-offset_failure]*100,1,'r','filled');

%Plot simulated data (not mirrored)
% scatter(angvel_success,offset_success*100,5,[0,.7,0],'filled');
% scatter(angvel_failure,offset_failure*100,5,'r','filled');

% Load the data
%load('Offset Tests with Rotation Landing Envelope 12-Sep-2015.mat')
load('Skew Tests with Rotation and Offset Landing Envelope 11-Sep-2015.mat')
offset_success_data = envelope.success(:,4);
angvel_success_data = envelope.success(:,3);
offset_failure_data = envelope.failure(:,4);
angvel_failure_data = envelope.failure(:,3);

% %mirrored data
% scatter([-angvel_success_data;angvel_success_data],[offset_success_data;-offset_success_data]*100,40,[0,.7,0],'o');
% scatter([-angvel_failure_data;angvel_failure_data],[offset_failure_data;-offset_failure_data]*100,90,'r','x');
% scatter([-angvel_success_data;angvel_success_data],[offset_success_data;-offset_success_data]*100,1,[0,.7,0],'filled');
% scatter([-angvel_failure_data;angvel_failure_data],[offset_failure_data;-offset_failure_data]*100,20,'r','filled');

%not mirrored data
scatter(angvel_success_data,-offset_success_data*100,40,[0,.7,0],'o','LineWidth',2)
scatter(angvel_failure_data,-offset_failure_data*100,90,'r','x','LineWidth',2);
legend('success','failure')
scatter(angvel_success_data,-offset_success_data*100,1,[0,.7,0],'filled');
scatter(angvel_failure_data,-offset_failure_data*100,20,'r','filled');


%Plot red and green regions
h1 = patch([-998,1200,1200,-998],[-13,-13,13,13],[1,.95,.95]);
h2 = patch(x_bound,y_bound,[.95,1,.95],'EdgeColor',[0,.7,0]);

% Plot axes
line([0 0], [-13,13],'Color',[0,0,0],'LineWidth',1);  %x-axis
line([-1000,1000], [0 0],'Color',[0,0,0],'LineWidth',1);  %y-axis
% line([-1000,-1000], [-13,13],'Color',[0,0,0],'LineWidth',1);  %y-axis

%Draw Gripper Limit
xL = [-1000,1000];
line(xL, [13,13],'Color',[1,0,0],'LineWidth',3);  %Gripper Limit
line(xL, [-13,-13],'Color',[1,0,0],'LineWidth',3);  %Gripper Limit

xlim(xL);
ylim([-13.1,13.1]);

xlabel('Angular velocity (deg/s)')
ylabel('Offset (cm)')

uistack(h2,'bottom');
uistack(h1,'bottom');

%grid on