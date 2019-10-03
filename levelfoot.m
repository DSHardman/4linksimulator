function pos = levelfoot(pos,corners)
%%moves foot just above the obstacle to which its height is closest
%prevents collision being flagged when foot is on obastacle

%account for ground as an option
corners = [[0;0] [0;0] corners]; 

distances = [];
for i = 1:length(corners)/2
    distances = [distances; abs(pos(2)-corners(2,2*i))];
end

[~,loc] = min(distances);

newheight = corners(2,2*loc);
pos(2) = newheight + 0.01;
end