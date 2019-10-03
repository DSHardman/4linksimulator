function[smoothout] = curve1(start, finish, p)
%returns array of third order polynomial given the start and end points

smoothout = zeros(p,1);
for i = 1:1:p
    x = start + (i/p)*(finish-start);
    if x > 1
        x = 1;
    end
    if x < 0
        x = 0;
    end
    
    smoothout(i) = x;
    
end
end