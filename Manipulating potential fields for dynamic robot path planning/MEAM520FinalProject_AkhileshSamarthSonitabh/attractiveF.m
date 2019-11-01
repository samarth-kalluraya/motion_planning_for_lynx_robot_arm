% The function 'attractiveF.m' takes input as zeta, d, current joint
% position and the goal position. It gives the F_att matrix and the
% distance of the end effector from its goal position.
function [F_att, dist] = attractiveF(zeta, d, jointPositionCurrent, jointPositionGoal)

F_att = [];
% We first calculate the distance of the end effector in its current
% position from the goal position using the norm function
for i = 2:6
    dist(i-1) = norm(jointPositionCurrent(i,1:3)-jointPositionGoal(i,1:3));
end

% Depending on the distance of the current end effector position from the
% goal, we use conic well potential or parabolic well potential to
% calculate the attractive force acting on the joints of the Lynx. If the
% position is at a distance less than 'd', we use parabolic potential.
% Else, we use conic well potential
for i = 1:5
    if dist(i) <= d
        F_att(i,:) = -1*zeta(i)*(jointPositionCurrent(i,1:3)-jointPositionGoal(i,1:3));
    else
        F_att(i,:) = -1*d*zeta(i)*((jointPositionCurrent(i,1:3)-jointPositionGoal(i,1:3))/dist(i));
    end
end
end
        