%% Code integration
% CV code needs to be run twice to detect both the sequences
% original_order = CV();
% final_order = CV();

% lynxStart('Hardware','Legend','Port','COM3')
lynxStart('Hardware', 'off', 'Frame', 'on', 'Shadow', 'off','Gripper','on')

% The following 2 lines of code can be uncommented to check the code
% % without running the CV code.
original_order = ["blue","pink","green","red","yellow"];
final_order = ["yellow","red","green","pink","blue"];

indexPos = arrange(original_order, final_order);
indexMat = cell2mat(struct2cell(indexPos));
sz = size(indexMat,3);
indPos = reshape(indexMat,[2,sz])';

% We store the joint angles of all the block positions in a struct so that
% they can be accessed easily when required
blockQ = struct;

blockQ(1).q = [-0.7532,0.4440,0.1371,0.9897,0.7532,20];
blockQ(2).q = [-0.5124,0.2359,0.4113,0.9236,-0.5124,20];
blockQ(3).q = [-0.1853,0.1085,0.5603,0.9021,-0.1853,20];
blockQ(4).q = [0.1853,0.1085,0.5603,0.9021,0.1853,20];
blockQ(5).q = [0.5124,0.2359,0.4113,0.9236,0.5124,20];
blockQ(6).q = [0.7532,0.4440,0.1371,0.9897,0.7532,20];


s = size(blockQ,2);
a = randperm(s);
a = sort(a);

mainmap = loadmap("map_6.txt");
map = struct;
i = 1;
j = 1;
%% Maps generation

% We use this part of the code to generate different maps for each of the
% different movements of the Lynx. We first breakdown the each step of the
% sorting struct into 2 steps, i.e. to reach the position of the first
% block and then move the block to the final position
while i < 2*sz+1
    startp1 = indPos(j,1);
    goalp1 = indPos(j,2);
    order(i,1) = startp1;
    order(i,2) = goalp1;
    b = a;
    b(find(b==startp1)) = [];
    b(find(b==goalp1)) = [];
    row(i,:) = b;
    startp2 = indPos(j,2);
    if j<sz
        goalp2 = indPos(j+1,1);
        order(i+1,1) = startp2;
        order(i+1,2) = goalp2;
        b=a;
        b(find(b==startp2)) = [];
        b(find(b==goalp2)) = [];
        row(i+1,:) = b;
        i = i+2;
        j = j+1;
    else
        break
    end
end

% We then generate maps for each of these set of movements. For each set of
% movement, the obstacles change and hence the repulsive fields on the Lynx
for i = 1:size(row,1)
    map(i).obstacles(1,:) = mainmap.obstacles(row(i,1),:);
    map(i).obstacles(2,:) = mainmap.obstacles(row(i,2),:);
    map(i).obstacles(3,:) = mainmap.obstacles(row(i,3),:);
    map(i).obstacles(4,:) = mainmap.obstacles(row(i,4),:);
    map(i).obstacles(5,:) = mainmap.obstacles(7,:);
    map(i).obstacles(6,:) = mainmap.obstacles(8,:);
    map(i).obstacles(7,:) = mainmap.obstacles(9,:);
    map(i).boundary = mainmap.boundary;
end

%% Lynx Movement

% Move the Lynx from its 0 position to the position of the first block of
% the order using potential field
startq = [-0.7, 0.35, 0.1, 1.2, -0.7,30];
goalp = indPos(1,1);
goalq = blockQ(goalp).q;
goalq(6) = 30;
b=a;
b(find(b==goalp)) = [];
r(1,:) = b;
maping.obstacles(1,:) = mainmap.obstacles(r(1,1),:);
maping.obstacles(2,:) = mainmap.obstacles(r(1,2),:);
maping.obstacles(3,:) = mainmap.obstacles(r(1,3),:);
maping.obstacles(4,:) = mainmap.obstacles(5,:);
maping.obstacles(5,:) = mainmap.obstacles(6,:);
maping.obstacles(6,:) = mainmap.obstacles(7,:);
maping.boundary = mainmap.boundary;

potField(startq,goalq,maping);
pause(1);

% Close the gripper to pick the block
lynxServo(goalq(1), goalq(2),goalq(3),goalq(4),goalq(5),20);

% This for loop executes the remaining set of arrangements to arrange the
% original order into the final order
q6 = 20;
tic;
% The simulations can be run by remove the pauses and using lynxServoSim
% here as well as in the potField.m file.
% The program can be run on the Lynx by using lynxServo along with the
% pause functions. These changes need to be made in both mainFunction.m and
% potField.m
for i = 1:size(order,1)
    startp = order(i,1);
    goalp = order(i,2);
    startq = blockQ(startp).q;
    startq(6) = q6;
    goalq = blockQ(goalp).q;
    goalq(6) = q6;
%     lynxServo(startq);
    maps = map(i);
    f = plotmap(maps);
    LynxPath(i).path = potField(startq,goalq,maps);
%     pause(2);
    if rem(i,2) == 1
        q6 = 30;
        lynxServo(goalq(1), goalq(2),goalq(3),goalq(4),goalq(5),20);
%         pause(1);
        disp("obstacle placed");
    else
        q6 = 20;
        lynxServo(goalq(1), goalq(2),goalq(3),goalq(4),goalq(5),30);
%         pause(1);
        disp("obstacle picked");
    end
end
toc;