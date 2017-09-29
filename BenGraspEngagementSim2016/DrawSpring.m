function [handle] = DrawSpring(x1,y1,x2,y2,Width,Ncoils,LnoCoils)

    dx = x2 - x1;
    dy = y2 - y1;
    L = sqrt(dx^2+dy^2);
    Lcoils = L - LnoCoils;
    
    xcoils = [0, .25:.5:(Ncoils-.25), Ncoils]*Lcoils/Ncoils;
    ycoils = [ones(1,Ncoils); -ones(1,Ncoils)];
    ycoils = [0, Width/2*ycoils(:)', 0];
    x = [0, xcoils+LnoCoils/2, L];
    y = [0, ycoils, 0];
    points = [x;y];
    
    angle = atan2(dy,dx);
    dcm = angle2dcm(angle,0,0);
    dcm = dcm(1:2,1:2);
    
    points = dcm'*points + repmat([x1;y1],1,length(points(1,:)));
    
    handle = plot(points(1,:),points(2,:),'k','LineWidth',2);
    

end

