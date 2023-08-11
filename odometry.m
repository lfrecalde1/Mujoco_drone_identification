function [h,hp] = odometry(odom)
%UNTITLED Odometry of the Aerial Vehicle
% Read odometry values from ros
odomdata = receive(odom,3);

pose = odomdata.Pose.Pose;
vel = odomdata.Twist.Twist;

quat = pose.Orientation;
%angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

% Get values of position an orientation
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;


% Get values of linear an angular velocities
vx = vel.Linear.X;
vy = vel.Linear.Y;
vz = vel.Linear.Z;


wx = vel.Angular.X;
wy = vel.Angular.Y;
wz = vel.Angular.Z;

% create vector of position and angular states
h = [x;...
     y;...
     z;...
     quat.W;...
     quat.X;...
     quat.Y;...
     quat.Z];

% Create vector of linear an angular velocities
hp = [vx;...
      vy;...
      vz;...
      wx;...
      wy;...
      wz];

end

