% We use this function to check if the new set of joint angles generated
% lie within the joint limits for each joint. It takes the current set of
% joint angles as the input and gives the updated set of joint angles as
% the output.
function [q1, q2, q3, q4, q5, q6] = checkLinkLimits(q)

qmin = [-1.4 -1.2 -1.8 -1.9 -2 -15];        % define the maximum and the minimum joint angles for each joint
qmax = [1.4 1.4 1.7 1.7 1.5 30];

% If the current joint angle is more than the maximum limit or less than
% the minimum limit, we update the joint angle to the maximum value or the
% minimum value of the joint angle respectively. Else, if the values we are
% within limit, we leave it the same 
if q(1) < qmin(1)
    q1 = qmin(1);
elseif q(1) > qmax(1)
    q1 = qmax(1);
else
    q1 = q(1);
end

if q(2) < qmin(2)
    q2 = qmin(2);
elseif q(2) > qmax(2)
    q2 = qmax(2);
else
    q2 = q(2);
end

if q(3) < qmin(3)
    q3 = qmin(3);
elseif q(3) > qmax(3)
    q3 = qmax(3);
else
    q3 = q(3);
end

if q(4) < qmin(4)
    q4 = qmin(4);
elseif q(4) > qmax(4)
    q4 = qmax(4);
else
    q4 = q(4);
end

if q(5) < qmin(5)
    q5 = qmin(5);
elseif q(5) > qmax(5)
    q5 = qmax(5);
else
    q5 = q(5);
end

if q(6) < qmin(6)
    q6 = qmin(6);
elseif q(6) > qmax(6)
    q6 = qmax(6);
else
    q6 = q(6);
end

