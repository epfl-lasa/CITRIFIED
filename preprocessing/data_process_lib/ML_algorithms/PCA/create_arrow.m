function []=create_arrow(f_nb,startingpoint,vector,l,clr,linewidth)

d=length(vector);

if length(vector)<3
    ex=[startingpoint(1) startingpoint(1)+l*vector(1)];
    ey=[startingpoint(2) startingpoint(2)+l*vector(2)];
    figure(f_nb)
    hold on
    plot(ex,ey,'Color',clr,'Linewidth',linewidth)
    plot([startingpoint(1)+l*vector(1) startingpoint(1)+l*vector(1)-(l/4)*cos(atan(vector(2)/vector(1))-pi/14.4)],[startingpoint(2)+l*vector(2) startingpoint(2)+l*vector(2)-(l/4)*sin(atan(vector(2)/vector(1))-pi/14.4)],clr,'Linewidth',linewidth)
    plot([startingpoint(1)+l*vector(1) startingpoint(1)+l*vector(1)-(l/4)*cos(atan(vector(2)/vector(1))+pi/14.4)],[startingpoint(2)+l*vector(2) startingpoint(2)+l*vector(2)-(l/4)*sin(atan(vector(2)/vector(1))+pi/14.4)],clr,'Linewidth',linewidth)
    
else
    ex=[startingpoint(1) startingpoint(1)+l*vector(1)];
    ey=[startingpoint(2) startingpoint(2)+l*vector(2)];
    ez=[startingpoint(3) startingpoint(3)+l*vector(3)];
    figure(f_nb)
    hold on
    plot3(ex,ey,ez,'Color',clr,'Linewidth',linewidth)
    
end









end