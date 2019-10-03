function index = findindices(statespace,state,n)
%%returns the index of a particular state in the statespace of base-n
%%expressions.
%States range from 1 -> n^3.

index = 0;

startbase = dec2base(state-1,n,3);

for i = 1:length(statespace)
   if statespace(i,:) == startbase
      index = i; 
   end
end

if index == 0
    fprintf('STATE NOT IN STATESPACE: %d\n',state)
end
end