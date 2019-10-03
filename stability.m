function [unstable] = stability(com,pos,gnd,lVec,fixed)
%%Checks whether robot is stable on the foot set as 'fixed'.
%gnd =/= 0 indicates an overhang of the foot from obstacle, which is also
%checked.

unstable = 0;
%Foot 1 fixed
if fixed == 1
    %No overhang
    if gnd == 0
       if (com(1) < pos(1)) || (com(1) > (pos(1)+lVec(5)))
           unstable = 1;
           fprintf('Unstable on foot 1\n')
       end
    end
    %Overhang cases
    if gnd > 0
        if (com(1) < pos(1)) || (com(1) > (pos(1)+gnd))
           unstable = 1;
           fprintf('Unstable on foot 1\n')
        end
    end
    if gnd < 0
        if (com(1) < pos(1)-gnd) || (com(1) > (pos(1)+lVec(5)))
           unstable = 1;
           fprintf('Unstable on foot 1\n')
        end
    end
           
end
%Foot 2 fixed
if fixed == 2
    %No overhang
   if gnd == 0
        if (com(1) > pos(1)) || (com(1) < (pos(1)-lVec(6)))
           unstable = 1;
           fprintf('Unstable on foot 2\n')
        end
   end
   %Overhang cases
   if gnd > 0
       if (com(1) > pos(1)) || (com(1) < (pos(1)+gnd-lVec(6)))
           unstable = 1;
           fprintf('Unstable on foot 2\n')
        end
   end
   if gnd < 0
       if (com(1) > pos(1)+gnd) || (com(1) < (pos(1)-lVec(6)))
           unstable = 1;
           fprintf('Unstable on foot 2\n')
        end
   end
end
end