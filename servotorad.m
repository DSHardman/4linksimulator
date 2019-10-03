function [rad] = servotorad(servoin,minangle)
%converts servo instruction to corresponding angle in radians
rad = servoin*2*pi/3 + minangle;
end