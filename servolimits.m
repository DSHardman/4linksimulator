function [minangle,maxangle] = servolimits(radangle,input)
%returns servo limits in radians given a known input and corresponding
%angle
minangle = radangle - (input*2*pi/3);
maxangle = radangle + ((1-input)*2*pi/3);
end