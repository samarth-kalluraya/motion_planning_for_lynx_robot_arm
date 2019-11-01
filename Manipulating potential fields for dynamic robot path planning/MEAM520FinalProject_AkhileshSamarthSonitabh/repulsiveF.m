% This function calculates the repulsive forces acting on each joint due to
% each of the obstacle in the workspace. It takes the values of eta, rho,
% obstacle boundaries and the current joint positions as the inputs. It
% gives the 3D force matric of repulsive forces and the minimum distance of
% the set of joints from the obstacles as the two outputs

function [F_rep,mindist] = repulsiveF(eta, rho, jointPositionCurrent, new_obs)

grid = size(new_obs);                 % gives the size of the array obstacles as m*n    
num_obstacles = grid(1);              % stores the number of obstacles

% In this set of for loops, we calculate the repulsive forces for each
% joint due to each of the obstacles. We thus vary i from 1 to the number
% of obstacles in the map.
for i = 1:num_obstacles
    % We first use the 'distPointToBox.m' function provided by the
    % instructor to calculate the distance of the joint angle from the
    % closest point on the obstacle. Again, we do this only for joints 2 to
    % 6 as joint 1 does not move at all.
    for j = 2:6
        dist(j-1) = distPointToBox(jointPositionCurrent(j,:),new_obs(i,:));
    end
    mdist(1,i) = min(dist);
    box_center = (new_obs(i,1:3)+new_obs(i,4:6))/2;
    
    % We then store the repulsive forces of each obstacle on each of the
    % joints in a 3D force matrix. If the distance of the joint from the
    % obstacle is more than rho, the force on that joint due to that
    % obstacle is set to 0
    for k = 1:5
        if i<(num_obstacles-2)
            cost=dot((jointPositionCurrent(k,:)-box_center)/norm(jointPositionCurrent(k,:)-box_center),[0 0 1]);
            sint=sqrt(1-cost^2);
            angular_rho = rho/(0.05+sint);
            if dist(k) <= angular_rho
                F_rep(k,:,i) = eta(k)*((1/dist(k)) - (1/angular_rho))^2*((jointPositionCurrent(k,:)-box_center)/dist(k));
                zizu=((1/dist(k)) - (1/angular_rho^2))^2;
%                 if k==5
%                     disp ("block replusive");
%                     disp ([sint,dist(k),angular_rho]);
%                     disp (zizu);
%                 end
            else
                F_rep(k,:,i) = [0 0 0];
%                 if k==5
%                     disp ([000,sint,dist(k)]);
%                 end
            end
        elseif(i==(num_obstacles-2))  
            if dist(k) <= 10
                F_rep(k,:,i) = eta(k)*((1/dist(k)) - (1/10))*([0 0 1]);
%                 if k==5
%                     disp ("upward");
%                     disp ([111,sint,dist(k)]);
%                 end
            else
                F_rep(k,:,i) = [0 0 0];
%                 if k==5
%                     disp ("null");
%                     disp ([222,sint,dist(k)]);
%                 end           
            end
        elseif (i==(num_obstacles-1))
            if dist(k) <= 10
                F_rep(k,:,i) = eta(k)*((1/dist(k)) - (1/10))*([-1 0 0]);
%                 if k==5
%                     disp ("backward");
%                     disp ([123,sint,dist(k)]);
%                 end
            else
                F_rep(k,:,i) = [0 0 0];
%                 if k==5
%                     disp ("null");
%                     disp ([123213,sint,dist(k)]);
%                 end           
            end
        elseif (i==(num_obstacles))
            if dist(k) <= 10
                F_rep(k,:,i) = eta(k)*((1/dist(k)) - (1/10))*([1 0 0]);
%                 if k==5
%                     disp ("forward");
%                     disp ([4942143,sint,dist(k)]);
%                 end
            else
                F_rep(k,:,i) = [0 0 0];
%                 if k==5
%                     disp ("null");
%                     disp ([7799,sint,dist(k)]);
%                 end           
            end
        end
    end
end

mindist = min(mdist);
end
        
