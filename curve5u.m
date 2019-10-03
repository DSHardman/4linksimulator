function[smoothout] = curve5u(start, finish, n, t0)
%%returns array of fifth order polynomial given the start and end points
f = start;

coeff = [t0^5, t0^4, t0^3; 5*t0^4, 4*t0^3, 3*t0^2; 20*t0^3, 12*t0^2, 6*t0];
values5 = coeff^(-1)*[finish - start; 0; 0];
a = values5(1);
b = values5(2);
c = values5(3);


smoothout = zeros(n);
for i = 1:n
    x = a*(i*t0/n)^5 + b*(i*t0/n)^4 + c*(i*t0/n)^3 + f;
    smoothout(i) = x;
    
end
end