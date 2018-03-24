%% initialization of ROS

%rosinit('http://192.168.1.110:11311', 'NodeHost', '192.168.1.100') %see what is the port/IP number
rosinit('http://192.168.1.100:11311', 'NodeHost', '192.168.1.101')
%%

%[armur, msgur]=rosactionclient('/vel_based_pos_traj_controller/follow_joint_trajectory');
%[armur, msgur]=rosactionclient('/pos_based_pos_traj_controller/follow_joint_trajectory');
[armur, msgur]=rosactionclient('/follow_joint_trajectory');

jpub5=rossubscriber('/joint_states');

msgur.Trajectory.JointNames={'shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'}';


%% initialization of UR10 robot in position tjPoint3

tjPoint1=rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint1.Positions = jpub5.LatestMessage.Position;
%tjPoint1.Positions(1) = jpub5.LatestMessage.Position(3);
%tjPoint1.Positions(3) = jpub5.LatestMessage.Position(1);
tjPoint1.Velocities = zeros(1,6);
tjPoint1.TimeFromStart = rosduration(0);

tjPoint2=rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint2.Positions = [20 -90 -90 -180 -90 0].*pi./180; 
tjPoint2.Velocities = [0.3 0 0 0 0 0];
tjPoint2.TimeFromStart = rosduration(10.0);

tjPoint3=rosmessage('trajectory_msgs/JointTrajectoryPoint');
tjPoint3.Positions = [40 -90 -90 -180 -90 0].*pi./180;
tjPoint3.Velocities = [0.1 0 0 0 0 0];
tjPoint3.TimeFromStart = rosduration(15);


tjP=[tjPoint1, tjPoint2, tjPoint3];

msgur.Trajectory.Points = tjP;
sendGoal(armur, msgur);
