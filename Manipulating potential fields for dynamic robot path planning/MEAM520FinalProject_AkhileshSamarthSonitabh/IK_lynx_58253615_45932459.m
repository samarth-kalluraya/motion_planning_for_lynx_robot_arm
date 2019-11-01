
function [q, is_possible] = IK_lynx_58253615_45932459(T0e)
% Input:    T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)

% Outputs:  q - a 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) which
%               are required for the Lynx robot to reach the given 
%               transformation matrix T
% 
%           is_possible - a boolean set to true if the provided
%               transformation T is achievable by the Lynx robot, ignoring
%               joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here
% Lynx ADL5 constants in mm

%Notation for the transformation matrices - Tmn is the transformation
%matrix for going from frame 'm' to frame 'n'

d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 69.85; %wrist to base of gripper
lg = 25.4; %length of gripper

a = T0e(1,3)*T0e(2,4) - T0e(2,3)*T0e(1,4);     %mathematical check to see if the position is reachable

%The value for a does not always come exactly equal to 0 due to error in
%measuring the position of the targets as well as truncating the value of
%sin and cos to 4 decimals. In order to compensate this, we are
%rounding a to 0 up to 1 digits so that the program does not show an
%error if the value for a is very small. Only if the rounded value is 0,
%the program will continue on to calculate the values for theta. Otherwise
%it shows an error saying that the orientation of target cannot be reached
%by the end effector.
if round(a) == 0
    Mc = T0e(1:3,4)-(lg+d5)*T0e(1:3,3);     %calculates the position of the centre of the wrist
    
    %calculating the 2 values for theta1
    
    theta1_1=atan2(Mc(2),Mc(1));            
    theta1_2=pi+atan2(Mc(2),Mc(1));
    
    %Using cosine rule, we find the values for cos(theta3) and sin(theta3)
    m=-(Mc(1)^2+Mc(2)^2+(Mc(3)-d1)^2-a2^2-a3^2)/(2*a2*a3);
    mm=sqrt(1-m^2);
    
    if m < -1
        %disp("The target point is beyond the workspace");
        is_possible = false;
        q = 0;
        %disp(is_possible);
    else
        
        %calculating the 2 values for theta3
        theta3_1=atan2(m,mm);
        theta3_2=atan2(m,-mm);
    
        %calculating the 4 values for theta2
        theta2_1=atan2(sqrt(Mc(1)^2+Mc(2)^2), Mc(3)-d1) - atan2(a3*cos(theta3_1), a2-a3*sin(theta3_1));
        theta2_2=atan2(sqrt(Mc(1)^2+Mc(2)^2), Mc(3)-d1) - atan2(a3*cos(theta3_2), a2-a3*sin(theta3_2));
        theta2_3=atan2(-sqrt(Mc(1)^2+Mc(2)^2), Mc(3)-d1) - atan2(a3*cos(theta3_1), a2-a3*sin(theta3_1));
        theta2_4=atan2(-sqrt(Mc(1)^2+Mc(2)^2), Mc(3)-d1) - atan2(a3*cos(theta3_2), a2-a3*sin(theta3_2));
    
        %We form a matrix of all the 4 possible sets of values for theta1,
        %theta2 and theta3 to further check if they lie within the joint
        %limits
        Possible_angles=[theta1_1 theta2_1 theta3_1; theta1_1 theta2_2 theta3_2; theta1_2 theta2_3 theta3_1; theta1_2 theta2_4 theta3_2];
        round(Possible_angles,4);
        
        %This for loop check if the angles, theta1, theta2 and theta3
        %lie in the 3rd or the 4th quadrant. If so, we subtract 2pi from
        %them to get them in the 1st or the 2nd quadrant i.e. between -pi
        %to pi. For eg. 5 radians becomes -1.2832 radians
        for g = 1:4
            for h = 1:3
                if Possible_angles(g,h) > pi
                    Possible_angles(g,h) = Possible_angles(g,h) - (2*pi);
                end
            end
        end
                
        %This for loop check whether the angles theta1,theta2 and theta3
        %obtained above are within the joint limits of the robot. Only
        %the set of angles in which all are within the limits get stored in
        %a different matrix Angles123
        j = 1;
        for i = 1:4
            if Possible_angles(i,1)>=-1.4001 && Possible_angles(i,1)<=1.4001 && Possible_angles(i,2) >= -1.2001 && Possible_angles(i,2)<=1.4001 && Possible_angles(i,3)>=-1.8001 && Possible_angles(i,3)<=1.7001
                Angles123(j,:) = Possible_angles(i,:);
                j=j+1;
            end
        end
        
        l = j-1;
        if l == 0
            %disp("Some of theta1, theta2 or theta3 are beyond joint limits. Hence no solution possible");
            is_possible = false;
            %disp(is_possible);
            q = 0;
        else
        %This for loop carries out the orientation part of the inverse
        %kinematics problem. We first find T30 from the angles theta1,
        %theta2 and theta3 that we have obtained after all the checks. We
        %then multiply the inverse of T30 with T0e to get T63. Then, by
        %comparing the T63 matrix with the matrix obtained in Euler angles,
        %we find theta4 and theta5.
            for k = 1:l
                T10=[cos(Angles123(k,1)) 0 -sin(Angles123(k,1)) 0; 
                    sin(Angles123(k,1)) 0 cos(Angles123(k,1)) 0;
                        0 -1 0 d1;
                        0 0 0 1];
                T21=[sin(Angles123(k,2)) cos(Angles123(k,2)) 0 a2*sin(Angles123(k,2));
                        -cos(Angles123(k,2)) sin(Angles123(k,2)) 0 -a2*cos(Angles123(k,2));
                        0 0 1 0;
                        0 0 0 1];
                T32=[-sin(Angles123(k,3)) -cos(Angles123(k,3)) 0 -a3*sin(Angles123(k,3));
                        cos(Angles123(k,3)) -sin(Angles123(k,3)) 0 a3*cos(Angles123(k,3));
                        0 0 1 0;
                        0 0 0 1];
                T30 = T10*T21*T32; 

                T63 = inv(T30)*T0e;

                theta4 = atan2(T63(2,4),T63(1,4));

                %We check if the angles, theta4 and theta5 lie in the 3rd or
                %the 4th quadrant. If so, we subtract 2pi from them to get them
                %in the 2nd quadrant.
                if theta4 > pi
                    theta4 = theta4 - (2*pi);
                end

                theta5 = atan2(-T63(3,1),-T63(3,2));

                if theta5 > pi
                    theta5 = theta5 - (2*pi);
                end

                %In the following if-else loop, we check if theta4 and theta5
                %lie within the joint limits. If not, we display an error
                %saying that the angles are beyond limits and hence no solution
                %is possible
                if theta4>=-1.9001 && theta4<=1.7001 && theta5>=-2.0001 && theta5<=1.5001
                    q = [Angles123(k,1) Angles123(k,2) Angles123(k,3) theta4 theta5];
                    is_possible = true;
                    break;
                else
                    k=k+1;
                    %disp("Theta4 and theta5 are beyond joint limits. Hence no solution possible");
                    is_possible = false;
                    q = 0;
                    %disp(is_possible);
                end
            end
        end
    end
else
    %disp("The orientation of the target cannot be matched by the end effector. Hence solution is not possible");
    is_possible = false;
    q = 0;
    %disp(is_possible);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

