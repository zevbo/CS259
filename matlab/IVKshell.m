

% Inverse velocity kinematics to trace out some shape in a plane.
% This shell doesn't include the code for starting ROS, setting up the publisher and message variables, etc.
% Lines with "**" in the associated comment need to be completed.

w1 = 133;
w2 = 99.6;
l1 = 425;
l2 = 392;
h1 = -240;
h2 = 99.7;

b_1 = transpose([0, 1, 0, w1 + w2, 0, l1 + l2]);
b_2 = transpose([0, 0, 1, h2, -(l1 + l2), 0]);
b_3 = transpose([0, 0, 1, h2, -l2, 0]);
b_4 = transpose([0, 0, 1, h2, 0, 0]);
b_5 = transpose([0, -1, 0, -w2, 0, 0]);
b_6 = transpose([0, 0, 1, 0, 0, 0]);
% Any time step not too large should be fine in general:
delT = 0.25;
% If the robot is trying to move too fast between waypoints and freezes because a safety control kicks in, one easy way to deal with that is to increase the time step (which correspondingly decreases the commanded velocity between waypoints).

% Rotation between the world frame and the desired end-effector orientation:
R = [0 0 -1 ; 0 -1 0 ; -1 0 0];

% ** Number of waypoints in your shape:
N = 36;

waypoints = zeros(3,N);
% All points in the shape should be in the same plane, x=-550:
waypoints(1,:) = -550*ones(1,N);
% A good starting place, for the first waypoint, is here:
waypoints(2:3,1) = [-100; 0];
% ** Set the y and z positions of the rest of the waypoints (you could use a loop to create a geometric shape, read values from a file you've created separately, etc.):
waypoints(2:3,28) = [-200; -0.0];
waypoints(2:3,29) = [-199.96000007999976; -2.827010186005209];
waypoints(2:3,30) = [-198.82784085892376; -15.087288491711321];
waypoints(2:3,31) = [-192.58982639228526; -34.97793223768336];
waypoints(2:3,32) = [-176.64269724867316; -49.23001510537037];
waypoints(2:3,33) = [-156.97669695521927; -46.82380870815438];
waypoints(2:3,34) = [-140.01134671209138; -36.669023252117185];
waypoints(2:3,35) = [-124.61509751532525; -23.857727847283908];
waypoints(2:3,36) = [-110.02780840877519; -9.977262732586464];
waypoints(2:3,1) = [-89.97219159122481; 9.977262732586464];
waypoints(2:3,2) = [-75.38490248467475; 23.857727847283908];
waypoints(2:3,3) = [-59.98865328790862; 36.669023252117185];
waypoints(2:3,4) = [-43.02330304478072; 46.82380870815438];
waypoints(2:3,5) = [-23.35730275132684; 49.23001510537037];
waypoints(2:3,6) = [-7.4101736077147535; 34.97793223768336];
waypoints(2:3,7) = [-1.1721591410762358; 15.087288491711321];
waypoints(2:3,8) = [-0.03999992000024122; 2.827010186005209];
waypoints(2:3,9) = [0; 0.0];
waypoints(2:3,10) = [0; -0.0];
waypoints(2:3,11) = [-0.03999992000024122; -2.827010186005209];
waypoints(2:3,12) = [-1.1721591410762358; -15.087288491711321];
waypoints(2:3,13) = [-7.4101736077147535; -34.97793223768336];
waypoints(2:3,14) = [-23.35730275132684; -49.23001510537037];
waypoints(2:3,15) = [-43.02330304478072; -46.82380870815438];
waypointsv(2:3,16) = [-59.98865328790862; -36.669023252117185];
waypoints(2:3,17) = [-75.38490248467475; -23.857727847283908];
waypoints(2:3,18) = [-89.97219159122481; -9.977262732586464];
waypoints(2:3,19) = [-110.02780840877519; 9.977262732586464];
waypoints(2:3,20) = [-124.61509751532525; 23.857727847283908];
waypoints(2:3,21) = [-140.01134671209138; 36.669023252117185];
waypoints(2:3,22) = [-156.97669695521927; 46.82380870815438];
waypoints(2:3,23) = [-176.64269724867316; 49.23001510537037];
waypoints(2:3,24) = [-192.58982639228526; 34.97793223768336];
waypoints(2:3,25) = [-198.82784085892376; 15.087288491711321];
waypoints(2:3,26) = [-199.96000007999976; 2.827010186005209];
waypoints(2:3,27) = [-200; 0.0];

if 1

% Use your inverse kinematics function to determine corresponding joint angles:
theta = zeros(6,N);
% For the first waypoint, we provide a good guess for the corresponding pose; for the rest, the previous pose is a good guess for the next one
theta(:,1) = IKnum2([R waypoints(:,1); 0 0 0 1],[0 -pi/2 pi/2 -pi/8 pi/2 -pi]);
for i=2:N
  theta(:,i) = IKnum2([R waypoints(:,i); 0 0 0 1],theta(:,i-1));
end

% Now there are the two ways to proceed described in the lab handout. To use the usual position controller:
% Zev note: starting this at i=2 
for i=2:N
 commandlist(i) = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 commandlist(i).Positions = theta(:,i);
 if i<N
% ** Set the desired joint velocities at each waypoint up until the last:
   theta_diff = theta(:,i) - theta(:, i - 1);
   commandlist(i).Velocities = theta_diff / delT;
 else
% (but at the last waypoint, the joint velocity should be 0)
   commandlist(i).Velocities = zeros(6,1);
 end
% Setting the start times for the waypoints is slightly tricky because both Sec and Nsec need to be integers, and Nsec must be less than 1e9:
 commandlist(i).TimeFromStart.Sec = 3+floor((i-1)/round(1/delT));
 commandlist(i).TimeFromStart.Nsec = mod(i-1,round(1/delT))*delT*1e9;
end

shapeMsg = rosmessage(armCmd);
shapeMsg.Points = commandlist;
shapeMsg.JointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

% Now "send(armCmd,shapeMsg)" will execute your trajectory, if shapeMsg.Points = commandlist and shapeMsg.JointNames is set appropriately as usual.

% Or, to use the pure velocity controller:
else
% ** The rest of the code will be skipped unless this next line is changed to "if 1"

% Make sure velCmd, velMsg, and jSub have been created, as described in the handout:
velCmd = rospublisher('/joint_group_vel_controller/command');
velMsg = rosmessage(velCmd);
jSub = rossubscriber('joint_states');

for i=1:size(theta,2)-1
  % get current position:
  jMsg = receive(jSub);
  % switch the order of the joints in the message that returns, to match the actual joint order:
  % angles = jMsg.Position([3 2 1 4 5 6]);
  % curr_t = FKShell(angles);
  % curr_pos = curr_t(1:3,4);
  % ** set the velocity based on the next desired position and the actual current position:
  % j = body_j([b1 b2 b3 b4 b5 b6], theta);
  % pos_j = j(1:3, 1:6);
  % j_inv = tranpose(pos_j) * (j * transpose(pos_j)) ^ (-1);
  % diff = (waypoints(:,i) - curr_pos);
  v_desired = [];% (j_inv * diff) / delT;
  velMsg.Data = v_desired;%.Position([3 2 1 4 5 6]);
  % execute the velocity command:
  send(velCmd,velMsg)
  % wait until the next time step:
  pause(delT)
end

% stop the robot!
velMsg.Data = zeros(6,1);
send(velCmd,velMsg)
end

function j = body_j(screws, thetas)
    curr_t = eye(4);
    j = [];
    sz = size(screws);
    for i_ = 1:sz(2)
        i = sz(2) - i_ + 1;
        curr_transform = t_adjoint(curr_t);
        screw = screws(1:6, i);
        j = [(curr_transform * screw) j];
        curr_t = curr_t * expm_se3(screw, - thetas(i));
    end
end
