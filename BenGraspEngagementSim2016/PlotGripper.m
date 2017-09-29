function [handle] = PlotGripper(X,Gripper)

    handle = zeros(10,1);
    
    t = .01;
    xG = X(1);
    yG = X(2);
    theta = X(3);
    w = Gripper.w;
    L = Gripper.span;
    
    xGrip = [-t/2,w-t,w-t,w,w,w-t,w-t,-t/2];
    yGrip = [t/2,t/2,L/2,L/2,-L/2,-L/2,-t/2,-t/2];
    Grip = [xGrip;yGrip];
    
    dcm = angle2dcm(-theta,0,0);
    dcm = dcm(1:2,1:2);
    Grip = dcm'*Grip + repmat([xG;yG],1,length(Grip(1,:)));
    
    handle(1) = patch(Grip(1,:),Grip(2,:),[.5,.5,.5]);

    
    %plot spring and damper
    handle(2) = DrawSpring(-.21,.02,xG-.13,.02,.02,6,.02);
    handle(3:4) = DrawDamper(-.21,-.02,xG-.13,-.02,.02,.02,.04,.03);
    
    %plot plate
    handle(5) = patch([xG-.13,xG-.12,xG-.12,xG-.13],[-.05,-.05,.05,.05],[.5,.5,.5]);
    
    %draw hinge point
    handle(6) = scatter(xG,yG,20,'k','filled');
    
    %draw flexure
    xf = 0:.01:.12;
    a = yG/.12^2;
    yf = a*xf.^2;
    xf = xf - .12 + xG;
    handle(7) = plot(xf,yf,'k','LineWidth',4);
    
    %Draw torsional springs
    x1 = xG-.12;
    y11 = .03; y12 = -.03;
    xy21 = [w-t;.04];
    xy21 = dcm'*xy21 + [xG;yG];
    xy22 = [w-t;-.04];
    xy22 = dcm'*xy22 + [xG;yG];
    
    handle(8) = DrawSpring(x1,y11,xy21(1),xy21(2),.01,10,.04);
    handle(9) = DrawSpring(x1,y12,xy22(1),xy22(2),.01,10,.04);
    
    %Draw sweet spot
    xss = [w,w,w+Gripper.d_norm,w+Gripper.d_norm];
    yss = [-Gripper.d_tan/2,Gripper.d_tan/2,Gripper.d_tan/2,-Gripper.d_tan/2];
    xyss = [xss;yss];
    xyss = dcm'*xyss + repmat([xG;yG],1,length(xyss(1,:)));
    handle(10) = patch(xyss(1,:),xyss(2,:),'g','EdgeColor','none');

end

