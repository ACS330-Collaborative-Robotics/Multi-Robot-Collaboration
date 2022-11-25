x = 0.1;
y = 0.1;
z = 0.4;
a = deg2rad(0); % rot about z
b = deg2rad(180); % rot about y
c = deg2rad(0); % rot about x

joints = inverse_kinematics(x,y,z,a,b,c)

mover6 = importrobot('CPMOVER6.urdf');

config = homeConfiguration(mover6)

for i = 1:6
    config(i).JointPosition = joints(i);
end % for

getTransform(mover6, config, "link6", "base_link")
show(mover6, config)