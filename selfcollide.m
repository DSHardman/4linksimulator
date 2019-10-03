function [scollide] = selfcollide(rMat,footsep)
%%Returns 1 if any links intersect.
%Footsep is the minimum distance that the feet may be together without
%colldiing.

scollide = 0;

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

%Following code necessary for original setup, but not with addition of new feet
%{
%check foot 1 collisions
for i = 2:1:4
    [x,y] = intsct(rMat(:,6),rMat(:,1),rMat(:,i),rMat(:,i+1));
    if isonline(x,y,rMat(:,6),rMat(:,1)) && isonline(x,y,rMat(:,i),rMat(:,i+1))
        scollide = 1;
    end
end
%}


%check collisions of link 1 with links 3,4
for i = 3:1:4
    [x,y] = intsct(rMat(:,1),rMat(:,2),rMat(:,i),rMat(:,i+1));
    if isonline(x,y,rMat(:,1),rMat(:,2)) && isonline(x,y,rMat(:,i),rMat(:,i+1))
        scollide = 1;
        fprintf('link 1 collision\n')
    end
end
%check collisions of link 4 with link 2
[x,y] = intsct(rMat(:,4),rMat(:,5),rMat(:,2),rMat(:,3));
if isonline(x,y,rMat(:,4),rMat(:,5)) && isonline(x,y,rMat(:,2),rMat(:,3))
    scollide = 1;
    fprintf('link 4 collision\n')
end


%intersection of feet
[aa,ba] = lineeq(rMat(:,6),rMat(:,1));
[ab,bb] = lineeq(rMat(:,7),rMat(:,5));
if (round(aa,1) == round(ab,1)) && (round(ba,1) == round(bb,1))
    if sqrt((rMat(1,1)-rMat(1,5))^2 + (rMat(2,1)-rMat(2,5))^2) < footsep
        scollide = 1;
        fprintf('foot collision\n')
    end
end
end