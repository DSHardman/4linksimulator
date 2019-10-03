function obstenter(corner)
%%Draws rectangular obstacles defined by corner.
%corner uses [[bottom left] [top right]] format

rectObj = rectangle('FaceColor','k');
set(rectObj,'Position',[corner(1,1) corner(2,1)...
    (corner(1,2)-corner(1,1)) (corner(2,2) - corner(2,1))]);
end

%Old Obstacle Enter Method, rounding using inbuilt grid:
%{
function [obstgrid] = obstenter(corner,horarray,vertarray)
%rectangular obstacle - enter 2 corners: bottom left & top right

bottomleft = coordtogrid(corner(:,1),horarray,vertarray);
topright = coordtogrid(corner(:,2),horarray,vertarray);
obstgrid = [bottomleft topright];

blplottedcoord = gridtocoord(bottomleft,horarray,vertarray);
trplottedcoord = gridtocoord([topright(1)+1;topright(2)+1],horarray,vertarray);

rectObj = rectangle('FaceColor','k');
set(rectObj,'Position',[blplottedcoord(1) blplottedcoord(2)...
    (trplottedcoord(1)-blplottedcoord(1)) (trplottedcoord(2)-blplottedcoord(2))]);
end
%}
