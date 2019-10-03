function [flatground,gnd] = onground(rMat,foot,corners)
%%If foot is flat on ground or top of obstacle, then flatground is true.
%gnd = 0 if there is no overhang of foot from obstacle.
%Else it defines magnitude & direction of overhang.

    flatground = 0;
    gnd = 0;
    %set endpoints of foot to be checked
    switch foot
        case 1
            point = [1;6];
        case 2
            point = [7;5];
    end

    %check if foot is on ground
    if round(2*rMat(2,point(1))) == 0 && round(2*rMat(2,point(2))) == 0
        flatground = 1;
        fprintf('Foot %d on ground\n',foot)
    end
    
    %if obstacles are entered, check if foot is on these
    if nargin == 3
        for i = 1:length(corners)/2
            obsttop = corners(2,2*i);
            if round(5*rMat(2,point(1)))/5 == round(5*obsttop)/5 && round(5*rMat(2,point(2)))/5 == round(5*obsttop)/5 ...
                    && corners(1,2*i-1) < rMat(1,point(2)) && corners(1,2*i) > rMat(1,point(1))
                flatground = 1;
                fprintf('Foot %d on obstacle\n',foot)
                %return gnd value to calculate stability
                switch foot
                    case 1
                        if rMat(1,1) < corners(1,2*i-1)
                            gnd = rMat(1,1) - corners(1,2*i-1);
                        elseif rMat(1,6) > corners(1,2*i)
                            gnd = rMat(1,6) - corners(1,2*i);
                        end
                    case 2
                        if rMat(1,5) > corners(1,2*i)
                            gnd = corners(1,2*i) - rMat(1,5);
                        elseif rMat(1,7) < corners(1,2*i-1)
                            gnd = corners(1,2*i-1) - rMat(1,7);
                        end
                end
            end
        end
    end
end