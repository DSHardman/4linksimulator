function ocoll = obstaclecollide(rMat,corners)
%%checks for collision of robot with obstacles

ocoll = 0;

%local function: find equation of line given 2 points.
%y = ax + b
function [a,b] = lineeq(p1,p2)
    a = (p2(2)-p1(2))/(p2(1)-p1(1));
    b = p1(2) - a*(p1(1));
end

%local function: intersection of 2 lines
function [x,y] = intsct(p1,p2,q1,q2)
    if p1(1) == p2(1) && q1(1) ~= q2(1)
        x = p1(1);
        [a2,b2] = lineeq(q1,q2);
        y = a2*x+b2;
    elseif p1(1) ~= p2(1) && q1(1) == q2(1)
        x = q1(1);
        [a1,b1] = lineeq(p1,p2);
        y = a1*x+b1;
    else
        [a1,b1] = lineeq(p1,p2);
        [a2,b2] = lineeq(q1,q2);
        x = (b2-b1)/(a1-a2);
        y = a1*x+b1;
    end
end

%is the intersection point between the points defined on the line?
function [pointonline] = isonline(x,y,p1,p2)
    pointonline = 0;
    if ((x>=p1(1) && x<=p2(1)) || (x<=p1(1) && x>=p2(1))) && ...
            ((y>=p1(2) && y<=p2(2)) || (y<=p1(2) && y>=p2(2)))
        pointonline = 1;
    end
end

temprMat = [rMat(:,6) rMat(:,1) rMat(:,2) rMat(:,3) rMat(:,4) rMat(:,5) rMat(:,7)];

%check whether main links or feet collide with obstacle
for i = 1:6
    %left side of obstacle
    [x,y] = intsct(corners(:,1),[corners(1,1);corners(2,2)],temprMat(:,i),temprMat(:,i+1));
    if isonline(x,y,corners(:,1),[corners(1,1);corners(2,2)]) && isonline(x,y,temprMat(:,i),temprMat(:,i+1))
        ocoll = 1;
        fprintf('Collision with left side of obstacle\n')
    end
    %right side of obstacle
    [x,y] = intsct(corners(:,2),[corners(1,2);corners(2,1)],temprMat(:,i),temprMat(:,i+1));
    if isonline(x,y,corners(:,2),[corners(1,2);corners(2,1)]) && isonline(x,y,temprMat(:,i),temprMat(:,i+1))
        ocoll = 1;
        fprintf('Collision with right side of obstacle\n')
    end
    %top of obstacle
    %small leniency added to y values to account for errors in planning
    %when stepping onto obstacles
    [x,y] = intsct(corners(:,2),[corners(1,1);corners(2,2)],temprMat(:,i)+[0;0.05;0],temprMat(:,i+1)+[0;0.05;0]);
    if isonline(x,y,corners(:,2),[corners(1,1);corners(2,2)]) && isonline(x,y,temprMat(:,i),temprMat(:,i+1))
        ocoll = 1;
        fprintf('Collision with top of obstacle\n')
    end
    %bottom of obstacle
    [x,y] = intsct(corners(:,1),[corners(1,2);corners(2,1)],temprMat(:,i),temprMat(:,i+1));
    if isonline(x,y,corners(:,1),[corners(1,2);corners(2,1)]) && isonline(x,y,temprMat(:,i),temprMat(:,i+1))
        ocoll = 1;
        fprintf('Collision with bottom of obstacle\n')
    end
end

end