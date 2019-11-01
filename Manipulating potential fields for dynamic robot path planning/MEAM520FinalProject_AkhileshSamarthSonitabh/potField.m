% This function uses potential field to calculate the path from a given
% start point to a given goal point in a predefined map. It takes the start
% point joint angles, goal point joint angles and the predefined map as
% input. As output, it runs a simulation of the Lynx going from the start
% configuration to the goal configuration in the given map.
% While calling the function, for the map, we need to directly load the map
% into the variable. For example, a command can be as follows:
% potField([-1.2 0 0 0 0 0],[0.5 0 0 0 0 0],loadmap("map_3.txt"))

function path = potField(q_start, q_goal, map)
% map=loadmap("map_6.txt");

% Toe_start=[1 0 0 160;
% 0 -1 0 90;
% 0 0 -1 10;
% 0 0 0 1];
% 
% Toe_goal=[1 0 0 160;
% 0 -1 0 -30;
% 0 0 -1 10;
% 0 0 0 1];

% Toe_start=[0 0 1 150;
% 0 -1 0 0;
% 1 0 0 110;
% 0 0 0 1];
% 
% Toe_goal=[0 0 1 215;
% 0 -1 0 0;
% 1 0 0 110;
% 0 0 0 1];
% 
% lynxStart('Hardware', 'off', 'Frame', 'on', 'Shadow', 'off','Gripper','on')
% [q_start,is_possible1]=IK_lynx_58253615_45932459(Toe_start);
% [q_goal,is_possible2]=IK_lynx_58253615_45932459(Toe_goal);
% 
% q_start(6)=20;
% q_goal(6)=20;

[jointPositionStart, startconfig] = calculateFK_sol(q_start);
[jointPositionGoal, endconfig] = calculateFK_sol(q_goal);
qmin = [-1.4 -1.2 -1.8 -1.9 -2 -15];        % define the maximum and the minimum joint angles for each joint
qmax = [1.4 1.4 1.7 1.7 1.5 30];
% plotmap(map);
               
obstacles = map.obstacles;                  % extracts the obstacles and boundary of the respective maps into variables
boundary = map.boundary;

grid = size(map.obstacles);                 % gives the size of the array obstacles as m*n    
num_obstacles = grid(1);                    % stores the number of obstacles
obs = [];
z = 1;

% This while loop extracts the obstacles from the map struct
while z<=num_obstacles
    obs(z,:) = map.obstacles(z,:);              
    z = z+1;
end

new_obs = [];

% The geometry of the lynx has been incorporated by adding a padding of 2.5cm
% so that the links do not collide with the obstacles.
width_lynx = 30;
x =1;

% This loop generates the new set of obstacles after adding padding to the
% obstacles in order to compensate for the geometry of the robot arm
while x<=num_obstacles
    new_obs(x,1) = obs(x,1) - width_lynx;
    new_obs(x,2) = obs(x,2) - width_lynx;
    new_obs(x,3) = obs(x,3) - width_lynx;
    new_obs(x,4) = obs(x,4) + width_lynx;
    new_obs(x,5) = obs(x,5) + width_lynx;
    new_obs(x,6) = obs(x,6) + width_lynx+20;
    x = x+1;
end

% In order to prevent self collision of the robot arm with its own links or
% its geometry, we have added another obstacle in the positions that we
% though would lead to self collision
% map.obstacles((num_obstacles+1),:) = [-100,-100,-200,100,100,30];
% new_obs((num_obstacles+1),:) = [-100,-100,-200,100,100,30];
% 
% We check if the start and goal points are not colliding with any of the
% obstacles using the 'check_linkcollision.m' function that we defined in
% Lab3. If the set of points is not colliding, only then we do the further
% calculations
Start = check_linkcollision(q_start,new_obs);
Goal = check_linkcollision(q_goal,new_obs);

if Start == 0 && Goal == 0
    
    % We calculate the distances of the joints from the start position to
    % the goal position. We do this only for joints 2 to 6 as position of
    % joint 1 does not change.
    for i = 2:6
        distToGoal(i-1) = norm(jointPositionStart(i,1:3)-jointPositionGoal(i,1:3));
    end
 %% Potential field calculations
    zeta = [1 1 1 10 10 1]*10000;
    eta = [1 1 1 1 1]*10000000;
    rho = 10;
    d = 30;

%% Path fomulation

    path = struct;
    path(1).q1 = q_start(1);
    path(1).q2 = q_start(2);
    path(1).q3 = q_start(3);
    path(1).q4 = q_start(4);
    path(1).q5 = q_start(5);
    path(1).q6 = q_start(6);
    path(1).isMinima = 0;       % stores if the point is at a local minima
    
    ctr = 1;                    % counter for number of points in the path
    alpha = 0.005;               % Variable for time step
    MinCtr = 0;                 % Counter to count the number of local minima encountered along the path
    qt = 0.01;                     % for generating random walk
    aaa=1;
    temp=[];
    
    % While the distance to goal position is greater than 5mm, we keep on
    % incrementing the joint angles using the attractive and repulsive
    % forces on each of the joint angles.
    while distToGoal(1) > 5 || distToGoal(2) > 5 || distToGoal(3)> 5 ...
            || distToGoal(4)> 5 || distToGoal(5) > 1
        currentPos = [path(ctr).q1, path(ctr).q2, path(ctr).q3, ...
            path(ctr).q4, path(ctr).q5, path(ctr).q6];

        [jointPositionCurrent, T0e] = calculateFK_sol(currentPos);
        
        % We calculate only the velocity jacobian as we do not need the
        % angular velocity components here. We also calculate the
        % attractive and repulsive forces on the Lynx in its current
        % position. We do this by using separate functions, 'VJacobian.m',
        % 'attractiveF.m' and 'repulsiveF.m'.
        [J1, J2, J3, J4, J5] = VJacobian(currentPos);
        [F_att, distToGoal] = attractiveF(zeta,d,jointPositionCurrent,jointPositionGoal);
        [F_rep, mindist(ctr,:)] = repulsiveF(eta, rho, jointPositionCurrent, new_obs);
        
        % We calculate the attractive joint torques due to the total
        % attractive forces acting on each joint and then add up the total
        % attractive torque on the arm
        T_att1 = J1'*F_att(1,:)';
        T_att2 = J2'*F_att(2,:)';
        T_att3 = J3'*F_att(3,:)';
        T_att4 = J4'*F_att(4,:)';
        T_att5 = J5'*F_att(5,:)';

        T_att = T_att1 + T_att2 + T_att3 + T_att4 + T_att5;
        
        % We calculate the repulsive torques on each of the joints due to
        % each of the obstacles by multiplying with the respective
        % Jacobians. We then add both attractive and repulsive torques to
        % get the total torque on the arm
        T_rep_Obs = [0; 0; 0; 0; 0];
        for i = 1:num_obstacles
            T_rep_Obs = T_rep_Obs + (J1'*F_rep(1,:,i)') + (J2'*F_rep(2,:,i)')+ (J3'*F_rep(3,:,i)') + (J4'*F_rep(4,:,i)') + (J5'*F_rep(5,:,i)');
        end
        T = T_att + T_rep_Obs;
        
        % Here, we check if the current position of the arm is a local
        % minima by using another function, 'checkMinima.m'. If it is a
        % local minima, we do a random walk to get it out of the minima.
        % Else, we calculate the next set of joint angles using the time
        % step and the torques acting on the arm. We also keep checking if
        % the set of joint angles generated lie within the joint limits. If
        % they exceed, we set them to the maximum or minimum limit
        % whichever is encountered.
        isLocalMinima = checkMinima(path);
        if isLocalMinima == 1
            while (1)
                disp("f");
                temp(1) = path(ctr).q1 + qt*(rand-0.5);
                temp(2) = path(ctr).q2 + qt*(rand-0.5);
                temp(3) = path(ctr).q3 + qt*(rand-0.5);
                temp(4) = path(ctr).q4 + qt*(rand-0.5);
                temp(5) = path(ctr).q5 + qt*(rand-0.5);
                temp(6) = q_start(6);
                collision_check=check_linkcollision(temp,new_obs);
                MaxLimit = [temp(1),temp(2),temp(3),temp(4),temp(5),temp(6)]<qmax;
                MinLimit = [temp(1),temp(2),temp(3),temp(4),temp(5),temp(6)]>qmin;
                aaa=aaa+1;
                if  min(MaxLimit) == 1 && min(MinLimit) == 1 && collision_check==0
                    path(ctr).q1 = temp(1);
                    path(ctr).q2 = temp(2);
                    path(ctr).q3 = temp(3);
                    path(ctr).q4 = temp(4);
                    path(ctr).q5 = temp(5);
                    path(ctr).q6 = temp(6);
                    break;
                elseif aaa==10
                    qt = qt*1.2;
                    aaa=0;
                    break;
                end
            end
            qt = qt*1.2;
            MinCtr = MinCtr+1;
            
        else
            path(ctr+1).q1 = path(ctr).q1 + alpha*(T(1)/norm(T));
            path(ctr+1).q2 = path(ctr).q2 + alpha*(T(2)/norm(T));
            path(ctr+1).q3 = path(ctr).q3 + alpha*(T(3)/norm(T));
            path(ctr+1).q4 = path(ctr).q4 + alpha*(T(4)/norm(T));
            path(ctr+1).q5 = path(ctr).q1;
            path(ctr+1).q6 = q_start(6);
            [path(ctr+1).q1,path(ctr+1).q2,path(ctr+1).q3,path(ctr+1).q4,path(ctr+1).q5,path(ctr+1).q6] = ... 
                checkLinkLimits([path(ctr+1).q1, path(ctr+1).q2, path(ctr+1).q3, path(ctr+1).q4, path(ctr+1).q5,path(ctr+1).q6]);
            ctr = ctr+1;
        end 

        path(ctr).isMinima = isLocalMinima;
        
        % We simulate the lynx with the generated set of points and also
        % plot the end effector point using the scatter3 function
        lynxServoSim(path(ctr).q1, path(ctr).q2, path(ctr).q3, path(ctr).q4, path(ctr).q5, path(ctr).q6)
        hold on
%         pause(0.2)
        [a, b] = calculateFK_sol([path(ctr).q1, path(ctr).q2, path(ctr).q3, path(ctr).q4, path(ctr).q5, path(ctr).q6]);
%         scatter3(b(1,4),b(2,4),b(3,4))
        hold on
%         disp([path(ctr).q1, path(ctr).q2, path(ctr).q3, path(ctr).q4, path(ctr).q5, path(ctr).q6]);
    end
    
    % We display the time taken for the simulation and the minimum distance
    % of the arm from the obstacles.
%     disp(min(mindist));
else
    if Start == 1
        disp("Start point is inside the obstacle");
    elseif Goal == 1
        disp("Goal point is inside the obstacle");
    end
end
end



