function [is_colliding] = check_linkcollision(q, new_obs)
a = 0;          % counter variables for checking if there is no collision of the link with the obstacle.
b = 0;
c = 0;

grid = size(new_obs);
num_obstacles = grid(1);

[jointPos, T0e] = calculateFK_sol(q);

% This for loop checks if the generated configuration leads to a collision
% of the links of the robot arm with the obstacles. We have not considered
% q5 here as we have assumed that the wrist is always straight.
for i = 1:num_obstacles
    
    a = a + detectcollision(jointPos(2,:),jointPos(3,:), new_obs(i,:));
    b = b + detectcollision(jointPos(3,:),jointPos(4,:), new_obs(i,:));
    c = c + detectcollision(jointPos(4,:),T0e(1:3,4)', new_obs(i,:));
end

if a == 0 && b == 0 && c == 0
    is_colliding = false;           % not colliding with any obstacle
else
    is_colliding = true;            % collides
end

end