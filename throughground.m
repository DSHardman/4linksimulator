function [ground,joint] = throughground(rMat)
%%Checks whether any of the links are (partially) below ground level.
%ground is a boolean value, & joint is an array of joints below ground.
ground = 0;
joint = [];
for i = 1:length(rMat)
    if round(rMat(2,i),1) < 0
        ground = 1;
        joint = [joint, i];
        %fprintf('Joint %d below ground\n',i)
    end
end