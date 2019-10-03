%%Hard-coded robot walking

for i = 1:5
    pos1 = [-2;0]+[(i-1)*(lVec(2)+lVec(3))*(1-sin(pi/3));0];
    %%Stride Out
    %lift joint 1
    updateangles([0.8;pi/3;pi/3],[pi/3;pi/3;pi/3],lVec,mVec,pos1,footsep,1)
    %lower joint 1, extend joint 2 & curl joint 3
    updateangles([1.3;0;pi/2],[0.8;pi/3;pi/3],lVec,mVec,pos1,footsep,1)
    %lower 1 & 2 to ground
    updateangles([pi/2;0;pi/2],[1.3;0;pi/2],lVec,mVec,pos1,footsep,1)

    pos2 = [pos1(1)+lVec(2)+lVec(3);0];
    %%Stride In
    %lift joint 3
    updateangles([pi/2;0;1.3],[pi/2;0;pi/2],lVec,mVec,pos2,footsep,2)
    %bring together
    updateangles([pi/3;pi/3;pi/3],[pi/2;0;1.3],lVec,mVec,pos2,footsep,2)
end