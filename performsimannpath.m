%%Animation of pathtaken matrix output by simulated annealing.

for i = 1:size(pathtaken,1)
   if mod(i,2) == 1
      performpath(pathtaken(i,find(pathtaken(i,:))),lVec,mVec,posVec(:,i),footsep,1,servomin,stepsize,n,corners)
   else
      performpath(pathtaken(i,find(pathtaken(i,:))),lVec,mVec,posVec(:,i),footsep,2,servomin,stepsize,n,corners)
   end
end