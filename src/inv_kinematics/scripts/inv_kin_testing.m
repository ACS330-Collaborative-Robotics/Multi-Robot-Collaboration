x = 0.1;
y = 0.1;
z = 0.4;
a = deg2rad(0); % rot about z
b = deg2rad(180); % rot about y
c = deg2rad(0); % rot about x

[j1 j2 j3 j4 j5 j6] = inverse_kinematics(x,y,z,a,b,c)

mover6 = importrobot('CPMOVER6.urdf');

config = homeConfiguration(mover6)

config(1).JointPosition = j1;
config(2).JointPosition = j2;
config(3).JointPosition = j3;
config(4).JointPosition = j4;
config(5).JointPosition = j5;
config(6).JointPosition = j6;

getTransform(mover6, config, "link6", "base_link")