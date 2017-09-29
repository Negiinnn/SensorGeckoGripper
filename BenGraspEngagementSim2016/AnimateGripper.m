function [] = AnimateGripper(Tout,Xout,Gripper,Object,speed)
    
    x_min = -.22;
    x_max = max(Xout(:,7)) + Object.R;
    y_min = min(min(Xout(:,2))-Gripper.span/2-.1, min(Xout(1,8))-Object.R);
    y_max = max(max(Xout(:,2))+Gripper.span/2+.1, max(Xout(1,8))+Object.R);
    
    figure('units','normalized','outerposition',[0 0 1 1])
    axis equal
    axis manual
    
%     xlim([x_min, x_max]);
%     ylim([y_min, y_max]);

    xlim([-.22, .5]);
    ylim([-.25, .25]);
    
    xlabel('x');
    ylabel('y');
    
    hold on
    %grid on    
    
    Dot = 0;
    
    patch([x_min,x_min+.01,x_min+.01,x_min],[-.05,-.05,.05,.05],[.5,.5,.5]);
  
    
    h1 = PlotGripper(Xout(1,:),Gripper);
    h2 = PlotObject(Xout(1,:),Object,Dot);

    pause(1);
    i_old = 1;
    tic
    while toc < Tout(end)/speed
        
        temp = find(Tout > toc*speed);
        i = temp(1);
        %delete(h1);
        delete(h2);
        Dot = Dot + (Tout(i)-Tout(i_old))*Xout(i,11);
        %h1 = PlotGripper(Xout(i,:),Gripper);
        h2 = PlotObject(Xout(i,:),Object,Dot);
        plot([Xout(i_old,7),Xout(i,7)], [Xout(i_old,8),Xout(i,8)],'k','LineWidth',1);

%         uistack(h1(1),'bottom');
        uistack(h2(1),'bottom');
        i_old = i;
        pause(.01)

    end
    
    

end

