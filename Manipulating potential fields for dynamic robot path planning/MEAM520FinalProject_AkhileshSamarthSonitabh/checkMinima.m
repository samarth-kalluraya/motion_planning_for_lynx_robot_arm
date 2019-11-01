% We use this function to check if the current set of joint angles is a
% local minima. This function take the entire path (as calculated till the
% point) as the input and gives a boolean value as the output saying if its
% a local minima or not.
function isLocalMinima = checkMinima(path)

checkMin =[];
eps = 0.0005;                % An epsilon value to judge whether its a local minima or not
n = size(path);
numpts = n(2);

% We check the current set of joint angles with 3 previous sets of angles.
% We first extract them into a different array
if numpts>4
    for i = 1:4
        checkMin(i,1) = path(numpts-i+1).q1;
        checkMin(i,2) = path(numpts-i+1).q2;
        checkMin(i,3) = path(numpts-i+1).q3;
        checkMin(i,4) = path(numpts-i+1).q4;
        checkMin(i,5) = path(numpts-i+1).q5;
        checkMin(i,6) = path(numpts-i+1).q6;
    end

    % A check to see if the differences of all the current joint angles
    % with the previous joint angles is less than epsilon
    check = (abs(checkMin(1,:)-checkMin(2,:))<eps) + ...
        (abs(checkMin(1,:)-checkMin(3,:))<eps) + ... 
        (abs(checkMin(1,:)-checkMin(4,:))<eps);
    
    % Since, we have 6 joint angles and we are checking the current point
    % with 3 previous point, we will have a maximum value of 18. This will
    % be obtained when differences of all the joint angles from the
    % previous joint angles is less than epsilon, which is when local
    % minima is encountered.
    if sum(check) == 18
        isLocalMinima = 1;
        [a, b] = calculateFK_sol([path(numpts).q1,path(numpts).q2, ...
            path(numpts).q3, path(numpts).q4, path(numpts).q5, path(numpts).q6]);
    else
        isLocalMinima = 0;
    end
else
    % We return a 0 if the number of points in the path is less than 4
    isLocalMinima = 0;    
end
