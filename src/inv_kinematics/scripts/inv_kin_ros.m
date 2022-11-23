rosshutdown
rosinit

sumclient = rossvcclient("gazebo/get_model_state","gazebo_msgs/GetModelState", "DataFormat", "struct");

sumreq = rosmessage(sumclient);
sumreq.ModelName = 'block2';
sumreq.RelativeEntityName = 'mover6_a';

sumresp = call(sumclient,sumreq,"Timeout",3);

x = sumresp.Pose.Position.X;
y = sumresp.Pose.Position.Y;
z = sumresp.Pose.Position.Z + 0.2;

a = deg2rad(0); % Yaw
b = deg2rad(180); % Pitch
c = deg2rad(0); % Roll

[j1, j2, j3, j4, j5, j6] = inverse_kinematics(x,y,z,a,b,c);
joints_pos = [j1, j2, j3, j4, j5, j6];
joints_pos_deg = rad2deg(joints)

pub = [rospublisher('/mover6_a/joint1_position_controller/command', 'std_msgs/Float64')
       rospublisher('/mover6_a/joint2_position_controller/command', 'std_msgs/Float64')
       rospublisher('/mover6_a/joint3_position_controller/command', 'std_msgs/Float64')
       rospublisher('/mover6_a/joint4_position_controller/command', 'std_msgs/Float64')
       rospublisher('/mover6_a/joint5_position_controller/command', 'std_msgs/Float64')
       rospublisher('/mover6_a/joint6_position_controller/command', 'std_msgs/Float64')];

for joint = 1:6
    msg = rosmessage(pub(joint));
    msg.Data = joints_pos(joint);
    send(pub(joint), msg)
end % for


rosshutdown