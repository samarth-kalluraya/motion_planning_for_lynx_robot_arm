In our folder, we have used various functions. The use of each function has been explained below

1. mainFunction.m

This function is used to integrate the different modules of our project. It calls CV.m to get the current and goal order. These are then gievn as inputs to arrange.m, which returns the steps to be taken by the links. According to the steps generated, we generate different maps to run potential fields to move the blocks from 
their current positions to their goal positions.

2. CV.m

We use this function to detect the sequence of the colored blocks using computer vision. It gives the current order of the blocks and the goal order of the blocks as the output. The code needs to be run twice - once to detect the sequence of the current order and then the goal order.

3. arrange.m

This function takes the current order of blocks and the goal order of the blocks as the input. It returns the order in which the Lynx must move the blocks to reach
from the current order to goal order

4. potField.m

This function uses potential field to calculate the path from a given start point to a given goal point in a predefined map. It takes the start point joint angles, goal point joint angles and the predefined map as input. As output, it runs a simulation of the Lynx going from the start configuration to the goal configuration in the given map. 

5. VJacobian.m

We use this function to calculate the velocity jacobians for each of the joint angles in the current position. Although the function takes the set of joint angles, q, as the input, we only return the velocity jacobians for joints 1 to 5 The velocity Jacobians have been calculated in a separate function and then only the matrices have been exported here so as to prevent using systems toolbox

6. attractiveF.m

The function 'attractiveF.m' takes input as zeta, d, current joint position and the goal position. It gives the F_att matrix and the distance of the end effector from its goal position

7. repulsiveF.m

This function calculates the repulsive forces acting on each joint due to each of the obstacle in the workspace. It takes the values of eta, rho, obstacle boundaries and the current joint positions as the inputs. It gives the 3D force matrix of repulsive forces and the minimum distance of the set of joints from the obstacles as the two outputs

8. checkMinima.m

This function checks if the current set of joint angles is a local minima. This function take the entire path (as calculated till the current point) as the input and gives a boolean value as the output saying if its a local minima or not

9. check_linkcollision.m

This function checks if the current set of joint angles is colliding with any of the obstacles. It takes the current set of joint angles and the matrix of obstacle boundaries as the input and returns a boolean value saying whether the point is colliding or not

10. checkLinkLimits.m

This function checks if the new set of joint angles generated lie within the joint limits for each joint. It takes the current set of joint angles as the input and gives the updated set of joint angles as the output

11. RepulsiveFunctionPlot.m

This function has been used to plot the two repulsive force functions that we have defined in 	our report. This is to see the difference of forces at different distances from the obstacle

Other previously used functions that have been used again are:
1. calculateFK_sol.m - to perform forward kinematics on a set of joint angles
2. IK_lynx_58253615_45932459.m - to perform inverse kinematics on a given transformation matrix
3. loadmap.p - to load the map struct
4. plotmap.p - to plot the predefined map
5. lynxStart.m - to start the simulation of the Lynx manipulator
6. lynxServoSim.m - to simulate the motion of the Lynx on the generated path

NOTE: The function simulates the movement of the Lynx across different block positions but does not simulate the movement of the blocks themselves