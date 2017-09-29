function [handle] = DrawDamper(x1,y1,x2,y2,Width,L1,L2,Ldamp)
    
    handle = zeros(2,1);

    dx = x2 - x1;
    dy = y2 - y1;
    L = sqrt(dx^2+dy^2);
    %Ldamp = L - LnoDamp;
    
    xarm1 = [0,L1,L1,L1+Ldamp,L1,L1,L1+Ldamp];
    yarm1 = [0,0,Width/2,Width/2,Width/2,-Width/2,-Width/2];
    arm1 = [xarm1;yarm1];
    
    xarm2 = [L-L2,L-L2,L-L2,L];
    yarm2 = [Width*.3,-Width*.3,0,0];
    arm2 = [xarm2;yarm2];
    
    
    angle = atan2(dy,dx);
    dcm = angle2dcm(angle,0,0);
    dcm = dcm(1:2,1:2);
    
    arm1 = dcm'*arm1 + repmat([x1;y1],1,length(arm1(1,:)));
    arm2 = dcm'*arm2 + repmat([x1;y1],1,length(arm2(1,:)));
    

    handle(1) = plot(arm1(1,:),arm1(2,:),'k','LineWidth',2);
    handle(2) = plot(arm2(1,:),arm2(2,:),'k','LineWidth',2);
    

end

