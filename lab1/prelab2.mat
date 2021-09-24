rosinit
armCmd = rospublisher('/scaled_pos_joint_traj_controller/command')
testMsg = rosmessage(armCmd);

p1 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
p1.Positions = [0 0 0 0 0 0];
p1.Velocities = [0 0 0 0 0 0];
p1.TimeFromStart.Sec = 5;

p2 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
p2.Positions = [-1 0 0 0 0 0];
p2.Velocities = [0 0 0 0 0 0];
p2.TimeFromStart.Sec = 10;

p3 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
p3.Positions = [-1 -1 0 0 0 0];
p3.Velocities = [0 0 0 0 0 0];
p3.TimeFromStart.Sec = 15;

p4 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
p4.Positions = [0 -1 0 0 0 0];
p4.Velocities = [0 0 0 0 0 0];
p4.TimeFromStart.Sec = 20;

p5 = rosmessage('trajectory_msgs/JointTrajectoryPoint');
p5.Positions = [0 0 0 0 0 0];
p5.Velocities = [0 0 0 0 0 0];
p5.TimeFromStart.Sec = 25;

testMsg.Points = [p p p p p];
testMsg.JointNames = {'shoulder_pan_joint',
'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
'wrist_2_joint', 'wrist_3_joint'};

send(armCmd,testMsg);