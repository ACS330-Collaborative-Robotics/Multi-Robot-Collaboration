function [j1, j2, j3, j4, j5, j6] = inverse_kinematics(x, y, z, a, b, c)
%INVERSE_KINEMATICS Inverse kinematics of 6DOF Mover6 Manipulator
%   a - yaw
%   b - pitch
%   c - roll

mover6 = importrobot('CPMOVER6.urdf');

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
%abc_tform = eye(4);

%xyz_tform = [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];
abc_tform(1, 4) = x;
abc_tform(2, 4) = y;
abc_tform(3, 4) = z;

tform = abc_tform;

ik = inverseKinematics('RigidBodyTree', mover6);

% First three are orientation, Second three are xyz
weights = [0.5 0.5 0.5 1 1 0.2];

initialGuess = mover6.homeConfiguration;

[configSoln, solnInfo] = ik('link6', tform, weights, initialGuess);
disp(solnInfo.Status)

show(mover6, configSoln)

[j1, j2, j3, j4, j5, j6] = configSoln.JointPosition;

end

