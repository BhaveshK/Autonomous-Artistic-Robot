function [distan] = distance(x2,y2,x1,y1)
%The distance function takes in x and y co-ordinates of two points as input
%and returns distance between them
       ydiff=y2-y1;
       xdiff=x2-x1;
       ydiffsq=ydiff*ydiff;
       xdiffsq=xdiff*xdiff;
       distan=(ydiffsq+xdiffsq)^0.5;
end

