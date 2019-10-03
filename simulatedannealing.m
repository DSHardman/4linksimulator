function [finalpath,path] = simulatedannealing(n,statespace,startindex,endindex,T,testspertemp,tfraction,tempnumber)
%%Aims to find most efficient valid path between start & endstates
%Note input is startindex/endindex, NOT startstate/endstate.
%This is the index of the desired state in the statespace of valid states
%finalpath contains the states
%path contains the indices

%Annealing parameters
if nargin == 4
    T = 1;
    testspertemp = 10;
    tfraction = 0.9;
    tempnumber = 10;
end

attemptlimit = 50000;

%find adjacent states & corresponding number of motor movements
[adjacentmat,motormat] = findadjacentstates(statespace);

%%MAIN ANNEALING PROCESS
%Determine an initial path
path = proposenewpath(adjacentmat,startindex,endindex,10*attemptlimit);
if isempty(path)
    error('No Valid Path Could Be Found')
end
score = testscore(motormat,path,adjacentmat)
originalscore = score;
minimumscore = score;

%Optimisation: choose 2 points in the path at random, and find a new path
%between them.
for i = 1:tempnumber
   for j = 1:testspertemp
       boundaries = randsample(1:length(path),2);
       boundaries = sort(boundaries);
       newsection = proposenewpath(adjacentmat,path(boundaries(1)),path(boundaries(2)),attemptlimit);
       if isempty(newsection)
           continue
       end
       proposedpath = [path(1:boundaries(1)-1); newsection; path(boundaries(2)+1:end)];
       
       proposedscore = testscore(motormat,proposedpath,adjacentmat)
       %store minimum score for comparison
       if proposedscore < minimumscore
           minimumscore = proposedscore;
       end
       
       %Always choose new path if score is lower
       if proposedscore < score
           path = proposedpath;
           score = proposedscore;
       %choose new path with same/higher score only a fraction of the time
       else
          accprob = exp(-proposedscore/T);
          p = randsample([1;0],1,true,[accprob;1-accprob]);
          if p
              path = proposedpath;
              score = proposedscore;
          end
       end
   end
   T = tfraction*T;
end

%Display information about path found in command window.
finalpath = pathnormalise(path,statespace,n)
fprintf('Original Score: %d\n',originalscore)
fprintf('Final Score: %d\n',score)
fprintf('Minimum Score: %d\n',minimumscore)

%%LOCAL FUNCTIONS
%function tests total motor motions of a given path
function score = testscore(motormat,path,adjacentmat)
    score = 0;
    for h = 1:length(path) - 1
       optionchosen = find(adjacentmat(path(h),:) == path(h+1));
       %Reward moving multiple motors at once rather than separately in
       %sucession
       switch motormat(path(h),optionchosen)
           case 1
               score = score + 1;
           case 2
               score = score + 1.5;
           case 3
               score = score + 2;
       end
    end
end

%convert index path used to that describing the states
function path = pathnormalise(indexpath,statespace,n)
path = [];
for h = 1:length(indexpath)
   path(h) = base2dec(statespace(indexpath(h),:),n) + 1; 
end
end

%return proposed path between 2 state indices, in a maximum of 'limit'
%steps
function proposed = proposenewpath(adjacentmat,startind,endind,limit)
%searchvec is originally all valid options from start state
searchvec = adjacentmat(startind,:);
pathmat = searchvec;
%limit placed on number of steps until limit
steps = 0;
while steps < limit
   
   searchvec2 = [];
   pathmat2 = [];
   
   %search through possible options
   for h = 1:length(find(searchvec))

       adjopt = adjacentmat(searchvec(h),:);
       adjopt2 = [];
       for k = 1:length(find(adjopt))
         adjopt2 = [adjopt2 adjopt(k)];
         pathmat2 = [pathmat2 [pathmat(:,h);adjopt(k)]];
       end
       searchvec2 = [searchvec2 adjopt2];
   end
   
   %If a state has been added more than once, choose one at random
   selections = [];
   for h = 1:length(searchvec2)
      searchindex = find(searchvec2 == searchvec2(h));
      if length(searchindex) > 1
          selections = [selections randsample(searchindex,length(searchindex) - 1)];
      end
   end
   searchvec2([selections]) = [];
   pathmat2(:,[selections]) = [];
   
   %each row of pathmat is the path followed to meet the corresponding
   %searchvec entry
   pathmat = pathmat2;
   
   %once searchvec expanded, check if end state is reached
   if ismember(endind,searchvec2)
      fprintf('path found\n')
      proposed = [startind;pathmat(:,find(searchvec2 == endind))];
      break
   end
   
   searchvec = searchvec2;
   steps = steps + 1;
   
end

%Display error if no valid path could be found in set number of iterations.
if steps == limit
    proposed = [];
    fprintf('limit reached: no path found\n')
end
end

end
