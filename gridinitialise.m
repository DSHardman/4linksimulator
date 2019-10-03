function [horarray,vertarray] = gridinitialise(horlimits,hornum,vertlimits,vertnum)
%%Draw grid bounded in x by horlimits & y by vertlimits.
%hornum & vertnum of squares in each direction.

%arrays returned contain coodinates of each square.

horstep = (horlimits(2)-horlimits(1))/hornum;
horarray = horlimits(1);
for i = 1:hornum
    horarray = [horarray,horlimits(1)+i*horstep];
    
end

vertstep = (vertlimits(2)-vertlimits(1))/vertnum;
vertarray = vertlimits(1);

for i = 1:vertnum
    vertarray = [vertarray,vertlimits(1)+i*vertstep];
end

for i = 1:length(horarray)
    lineObj.v = line(0,0,'color','k','LineWidth',1);
    set(lineObj.v,'xdata',[horarray(i),horarray(i)],'ydata',[vertlimits(1),vertlimits(2)]);
end

for i = 1:length(vertarray)
    lineObj.h = line(0,0,'color','k','LineWidth',1);
    set(lineObj.h,'xdata',[horlimits(1),horlimits(2)],'ydata',[vertarray(i),vertarray(i)]);
end