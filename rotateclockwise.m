function rMat = rotateclockwise(point,rMat,angle)
%%Rotates rMat clockwise by angle about point
    rotmat = [cos(angle) sin(angle); -sin(angle) cos(angle)];
    
    for i = 1:length(rMat)
        aboutzero = [rMat(1,i) - point(1); rMat(2,i) - point(2)];
       newloc = point + rotmat*aboutzero;
       rMat(1,i) = newloc(1);
       rMat(2,i) = newloc(2);
    end
end