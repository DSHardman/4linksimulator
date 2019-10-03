function[smoothout] = curve5(start, finish, p)
%returns array of fifth order polynomial given the start and end points
t0 = 1;

f = start;

coeff = [t0^5, t0^4, t0^3; 5*t0^4, 4*t0^3, 3*t0^2; 20*t0^3, 12*t0^2, 6*t0];
values5 = coeff^(-1)*[finish - start; 0; 0];
a = values5(1);
b = values5(2);
c = values5(3);


smoothout = zeros(p,1);
for i = 1:1:p
    x = a*(i*t0/p)^5 + b*(i*t0/p)^4 + c*(i*t0/p)^3 + f;
    if x > 1
        x = 1;
    end
    if x < 0
        x = 0;
    end
    smoothout(i) = x;
    
end
end