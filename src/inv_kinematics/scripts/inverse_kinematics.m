function [j1, j2, j3, j4, j5, j6] = inverse_kinematics(x, y, z, a, b, c)
%INVERSE_KINEMATICS Inverse kinematics of 6DOF Mover6 Manipulator
%   Function loads CPRMover6.urdf then computes the inverse kinematics using
%   iterative method.
%
%   INPUT:
%   x - x coordinate relative to robot
%   y - y coordinate relative to robot
%   z - z coordinate relative to robot
%   a - yaw
%   b - pitch
%   c - roll
%
%   OUTPUT:
%   Joint positions 1->6 starting from base_link, moving towards link6
%   j1 j2 j3 j4 j5 j6
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Import URDF and setup RigidBodyTree
mover6 = importrobot('CPRMOVER6.urdf');

% Setup rotation matrices - source: http://msl.cs.uiuc.edu/planning/node102.html
a_tform = [cos(a) -sin(a) 0 0;
           sin(a) cos(a) 0 0;
           0 0 1 0;
           0 0 0 1];

b_tform = [cos(b) 0 sin(b) 0;
           0 1 0 0;
           -sin(b) 0 cos(b) 0;
           0 0 0 1];

c_tform = [1 0 0 0;
           0 cos(c) -sin(c) 0;
           0 sin(c) cos(c) 0;
           0 0 0 1];

abc_tform = a_tform*b_tform*c_tform;

% Add xyz transform to rotation matrix
abc_tform(1, 4) = x;
abc_tform(2, 4) = y;
abc_tform(3, 4) = z;

% Final 4x4 transform from base_link to link6
tform = abc_tform;

% Initiate ik object
ik = inverseKinematics('RigidBodyTree', mover6);

% First three are orientation, Second three are xyz
weights = [0.5 0.5 0.5 1 1 0.2];

% Start with initial guess of robot position
initialGuess = mover6.homeConfiguration;

% Iteratively improve robot position until it reaches optimal position
[configSoln, solnInfo] = ik('link6', tform, weights, initialGuess);

% Display ik status - "best available" or "success"
disp(solnInfo.Status)

% Display plot of robot position
show(mover6, configSoln)

% Output final joint positions
[j1, j2, j3, j4, j5, j6] = configSoln.JointPosition;

end

