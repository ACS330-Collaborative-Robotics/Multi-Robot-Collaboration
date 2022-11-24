rosshutdown
rosinit

sub = rossubscriber('/command_pos', 'inv_kinematics/ArmPos', 'DataFormat','struct');

listening = 1;
while listening
    % Listen on topic /command_pos for max 10 seconds before timeout
    [response,status,statustext] = receive(sub,10)
    
    % IF a response is recieved from the subscriber
    if status
        x = response.X;
        y = response.Y;
        z = response.Z;
        
        a = response.A; % Yaw
        b = response.B; % Pitch
        c = response.C; % Roll

        ns = response.RobotNamespace;
        
        [j1, j2, j3, j4, j5, j6] = inverse_kinematics(x,y,z,a,b,c);
        joints_pos = [j1, j2, j3, j4, j5, j6];
        
        % Setup publishers for relevant robot using robot namespace
        pub = [rospublisher(append(ns,'/joint1_position_controller/command'), 'std_msgs/Float64')
               rospublisher(append(ns,'/joint2_position_controller/command'), 'std_msgs/Float64')
               rospublisher(append(ns,'/joint3_position_controller/command'), 'std_msgs/Float64')
               rospublisher(append(ns,'/joint4_position_controller/command'), 'std_msgs/Float64')
               rospublisher(append(ns,'/joint5_position_controller/command'), 'std_msgs/Float64')
               rospublisher(append(ns,'/joint6_position_controller/command'), 'std_msgs/Float64')];
        
        % Publish each joint position to the relevant topic
        for joint = 1:6
            msg = rosmessage(pub(joint));
            msg.Data = joints_pos(joint);
            send(pub(joint), msg)
        end % for

    end % if
end % while

rosshutdown