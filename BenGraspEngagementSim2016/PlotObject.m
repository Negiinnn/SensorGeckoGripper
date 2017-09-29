function [handle] = PlotObject(X,Object, Dot)
    
    handle = zeros(2,1);

    xO = X(7);
    yO = X(8);
    R = Object.R;
    
    Dot = mod(Dot,2*pi);
    
    ang=0:0.01:2*pi; 
    xp=R*cos(ang);
    yp=R*sin(ang);
    handle(1) = patch(xO+xp,yO+yp,[.3,.5,1],'LineWidth',3,'EdgeColor',[0,0,.5]);
    hold on
    handle(2) = plot([xO+R*.78*cos(Dot),xO-R*.78*cos(Dot)],...
                     [yO+R*.78*sin(Dot),yO-R*.78*sin(Dot)],'.','MarkerSize',50,'Color',[0,0,.5]);
   
end

