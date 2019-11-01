% We use this function to calculate the velocity jacobians for each of the
% joint angles. Although we take the set of joint angles, q, as the input,
% we only return the velocity jacobians for joints 1 to 5
% The velocity Jacobians have been calculated in a separate function and
% then only the matrices have been exported here so as to prevent using
% systems toolbox

function [Jac1, Jac2, Jac3, Jac4, Jac5] = VJacobian(q)

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 69.85; %wrist to base of gripper
lg = 25.4; %length of gripper

Jac4 = [ -sin(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2))), -cos(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2))), -a3*sin(q(2) + q(3))*cos(q(1)), 0, 0;
  cos(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2))), -sin(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2))), -a3*sin(q(2) + q(3))*sin(q(1)), 0, 0;
                                       0,          - a3*cos(q(2) + q(3)) - a2*sin(q(2)),         -a3*cos(q(2) + q(3)), 0, 0];
 
Jac3 = [ -sin(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2))), -cos(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2))), -a3*sin(q(2) + q(3))*cos(q(1)), 0, 0;
         cos(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2))), -sin(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2))), -a3*sin(q(2) + q(3))*sin(q(1)), 0, 0;
                                            0,          - a3*cos(q(2) + q(3)) - a2*sin(q(2)),         -a3*cos(q(2) + q(3)), 0, 0];

Jac2 = [ -a2*sin(q(1))*sin(q(2)), a2*cos(q(1))*cos(q(2)), 0, 0, 0;
            a2*cos(q(1))*sin(q(2)), a2*cos(q(2))*sin(q(1)), 0, 0, 0;
                           0,        -a2*sin(q(2)), 0, 0, 0];

Jac1 = [ 0, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0];
     
Jac5 = [ -sin(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2)) + (d5+lg)*cos(q(2) + q(3) + q(4))),   -cos(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2)) + (d5+lg)*sin(q(2) + q(3) + q(4))),    -cos(q(1))*(a3*sin(q(2) + q(3)) + (d5+lg)*sin(q(2) + q(3) + q(4))),   -(d5+lg)*sin(q(2) + q(3) + q(4))*cos(q(1)),                                  0;
         cos(q(1))*(a3*cos(q(2) + q(3)) + a2*sin(q(2)) + (d5+lg)*cos(q(2) + q(3) + q(4))),   -sin(q(1))*(a3*sin(q(2) + q(3)) - a2*cos(q(2)) + (d5+lg)*sin(q(2) + q(3) + q(4))),    -sin(q(1))*(a3*sin(q(2) + q(3)) + (d5+lg)*sin(q(2) + q(3) + q(4))),   -(d5+lg)*sin(q(2) + q(3) + q(4))*sin(q(1)),                                  0;
                                                                                        0,              - a3*cos(q(2) + q(3)) - a2*sin(q(2)) - (d5+lg)*cos(q(2) + q(3) + q(4)),               - a3*cos(q(2) + q(3)) - (d5+lg)*cos(q(2) + q(3) + q(4)),             -(d5+lg)*cos(q(2) + q(3) + q(4)),                                  0];
end
