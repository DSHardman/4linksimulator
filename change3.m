function [phiVec] = change3(joint,direction,phiVec,stepsize)
%%Updates phiVec for one joint.
%Direction 1: Increase angle.
%Direction 0: Decrease angle.
%Used in the Q-Learning script.

if direction == 1
    phiVec(joint) = phiVec(joint) + stepsize;
end

if direction == 0
    phiVec(joint) = phiVec(joint) - stepsize;
end

end