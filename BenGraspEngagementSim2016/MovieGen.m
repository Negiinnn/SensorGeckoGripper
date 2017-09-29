function [] = MovieGen(Tout,Xout,Gripper,Object,speed,filename)


    MovieTime = Tout(end)/speed;
    FrameRate = 30;
    Nframes = floor(MovieTime*FrameRate);
    
    writerObj = VideoWriter(filename,'MPEG-4');
    writerObj.FrameRate = FrameRate;
    open(writerObj);
    
    x_min = -.22;
    x_max = max(Xout(:,7)) + Object.R;
    y_min = min(min(Xout(:,2))-Gripper.span/2-.1, min(Xout(1,8))-Object.R);
    y_max = max(max(Xout(:,2))+Gripper.span/2+.1, max(Xout(1,8))+Object.R);
    
    figure('units','normalized','outerposition',[0 0 1 1])
    axis equal
    
%     xlim([x_min, x_max]);
%     ylim([y_min, y_max]); 
    
    xlim([-.3, .5]);
    ylim([-.205, .205]); 
    
    xlabel('x');
    ylabel('y');
    
    hold on
    
    Dot = 0;
    
    patch([x_min,x_min+.01,x_min+.01,x_min],[-.05,-.05,.05,.05],[.5,.5,.5]);
    
    h1 = PlotGripper(Xout(1,:),Gripper);
    h2 = PlotObject(Xout(1,:),Object,Dot);

    i_old = 1;
    for f = 1:Nframes+1
        i = find(Tout > (speed*(f-1)/FrameRate),1);
        delete(h1);
        delete(h2);
        Dot = Dot + (Tout(i)-Tout(i_old))*Xout(i,11);
        h1 = PlotGripper(Xout(i,:),Gripper);
        h2 = PlotObject(Xout(i,:),Object,Dot);
        
%         plot([Xout(i_old,7),Xout(i,7)], [Xout(i_old,8),Xout(i,8)],'k','LineWidth',1);
%         uistack(h1(1),'bottom');
%         uistack(h2(1),'bottom');
        i_old = i;
        
        frame = getframe;
        writeVideo(writerObj,frame);
    end
    
    close(writerObj);


end

