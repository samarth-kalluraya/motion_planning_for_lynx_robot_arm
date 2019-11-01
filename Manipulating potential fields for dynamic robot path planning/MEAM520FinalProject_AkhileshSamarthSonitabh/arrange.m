% og_order and goal_order will be given as a (nx1) vector to the function.
% The dimensions for both the vectors will be the same


function indexPos = arrange(original_order,final_order)

% original_order = ["red","yellow","green","blue","pink"];
% final_order = ["yellow","green","pink","red","blue"];

% Number the goal order so as to make arranging easy
l_goal = numel(final_order);
for i=1:l_goal
    goal_order(i) = i;
end

% Number the original order according to the goal order to arrange the
% original order
for i = 1:l_goal
    og_order(i) = find(original_order(i) == final_order);
end
rng(32);

t_order = og_order;

% Adds an extra position in both the goal order and the original order
og_order(numel(goal_order)+1) = 0;
goal_order(numel(goal_order)+1) = 0;

% Finds the length of the goal order and the original order
len_goal = numel(goal_order);
len_og = numel(og_order);

% The arrangement order has been stored in a struct
indexPos = struct;

% Calculates the distance of the position of each of the blocks in the
% original order from its position in the goal order
for i = 1:len_goal
    idx = find(goal_order == og_order(i));
    diff_order(i) = idx - i;
end

% Finds the block whose distance from its goal position is maximum
el = find(abs(diff_order) == max(abs(diff_order)));
zeroPos = find(og_order==0);

% If there are more than one element whose distance from goal position is
% maximum, we select the first one
if numel(el) > 1
    el = el(1);
end

% The element with maximum distance is shifted to the extra position which
% was added earlier. We then sort the remaining elements into their
% correct positions. The cases in which the positions of two elements are
% swapped, are solved by the next piece of code.
indexPos.currentPos = el;
indexPos.newPos = zeroPos;
og_order([el,zeroPos]) = og_order(fliplr([el,zeroPos]));
j = 2;

temp_order = og_order;

while (sum(og_order==goal_order)~=len_goal)
    zeroPos = find(og_order==0);
    actualEl = goal_order(zeroPos);
    actualElPos = find(og_order==actualEl);
    indexPos(j).currentPos = actualElPos;
    indexPos(j).newPos = zeroPos;
    og_order([zeroPos,actualElPos]) = og_order(fliplr([zeroPos,actualElPos]));
    j = j+1;
    if sum(temp_order==og_order) == len_goal
        j = j-1;
        break;
    end
    temp_order = og_order;
end

temp = (og_order(1:len_goal-1)==goal_order(1:len_goal-1));
% Here, the cases where positions of elements are swapped are arranged. In
% such a case, we select the first element of the two, place it in the
% extra position, then place the other element in its correct position
% before placing the former element in its goal position.
while sum(temp) ~= (len_goal-1)
    zeros = find(temp==0);
    disp_el = numel(zeros);
    zeroPos = find(og_order==0);
    zerosPos = find(og_order==zeros(1));
    og_order([zeroPos,zerosPos]) = og_order(fliplr([zeroPos,zerosPos]));
    indexPos(j).currentPos = zerosPos;
    indexPos(j).newPos = zeroPos;
    j = j+1;
    
    while j <= len_goal+1
        zeroPos = find(og_order==0);
        actualEl = goal_order(zeroPos);
        actualElPos = find(og_order==actualEl);
        indexPos(j).currentPos = actualElPos;
        indexPos(j).newPos = zeroPos;
        og_order([zeroPos,actualElPos]) = og_order(fliplr([zeroPos,actualElPos]));
        j = j+1;
        if indexPos(j-1).currentPos == indexPos(j-1).newPos
            j = j-1;
            break;
        end
        temp_order = og_order;
    end
    temp = (og_order(1:len_goal-1)==goal_order(1:len_goal-1));
end

% Checks if any extra moves have been made and deletes them. These
% generally occur only in the end
numSwitches = numel(indexPos);
for i = 1:numSwitches
    if indexPos(i).currentPos == indexPos(i).newPos && i == numSwitches
        indexPos(i).currentPos = [];
        indexPos(i).newPos = [];
    elseif indexPos(i).currentPos == indexPos(i).newPos
        for j=i:numSwitches-1
            indexPos(j).currentPos = indexPos(j+1).currentPos;
            indexPos(j).newPos = indexPos(j+1).newPos;
        end
    end
end

end