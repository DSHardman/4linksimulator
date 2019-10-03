function [adjacentmat,motormat] = findadjacentstates(statespace)
%for each state, find adjacent states (i.e. each motor is at most 1 step
%%different)
%returned in matrix form with each row corresponding to a state in the
%statespace - remaining space is filled with zeros.

adjacentmat = [];
motormat = [];
%i searches each state
for i = 1:length(statespace)
    adjacentstates = [];
    motormovements = [];
    %j compares each state against all others
   for j = 1:length(statespace)
       samemotor = 0;
       adjacentmotor = 0;
       %k compares each motor's position
       for k = 1:3
          if statespace(i,k) == statespace(j,k)
             samemotor = samemotor + 1; 
          end
          if statespace(i,k) == statespace(j,k) + 1 || statespace(i,k) == statespace(j,k) - 1
             adjacentmotor = adjacentmotor + 1;
          end
       end
       if samemotor + adjacentmotor == 3 && samemotor ~= 3
           adjacentstates = [adjacentstates j];
           motormovements = [motormovements adjacentmotor];
       end
   end
   
   %fill empty space with zeros so results can be concatanated
   zerofill = zeros(1,length(statespace)-1-length(adjacentstates));
   adjacentstates = [adjacentstates zerofill];
   motormovements = [motormovements zerofill];
   
   adjacentmat = [adjacentmat;adjacentstates];
   motormat = [motormat;motormovements];
end
end