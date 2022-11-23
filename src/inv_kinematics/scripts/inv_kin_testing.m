x = 0.1;
y = 0.1;
z = 0.4;

a = deg2rad(45); % Yaw
b = deg2rad(-45); % Pitch
c = deg2rad(60); % Roll

inverse_kinematics(x,y,z,a,b,c)