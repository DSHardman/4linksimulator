function[smoothout] = curve3(start, finish, p)
%returns array of third order polynomial given the start and end points
t0 = 1;
d = start;
a = 2*(start - finish)/(t0^3);
b = 3*(finish - start)/(t0^2);

smoothout = zeros(p,1);
for i = 1:1:p
    x = a*(i*t0/p)^3 + b*(i*t0/p)^2 + d;
    if x > 1
        x = 1;
    end
    if x < 0
        x = 0;
    end
    smoothout(i) = x;
    
end
end