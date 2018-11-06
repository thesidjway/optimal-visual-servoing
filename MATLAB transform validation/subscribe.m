i = 0;
poses_base = [];
poses_pan = [];
poses_tilt = [];
sub = rossubscriber('gazebo/link_states');
tilt_pub = rospublisher('/summit_xl_a/joint_tilt_position_controller/command','std_msgs/Float64');
pan_pub = rospublisher('/summit_xl_a/joint_pan_position_controller/command','std_msgs/Float64');

tilt_msg = rosmessage(tilt_pub);
pan_msg = rosmessage(pan_pub);

tilts = [0 0.5 1 1.5 1 0.5];
pans = [0.25 0.25 0.25 -0.25 1 -0.5];

for i = 1:size(tilts,2)
tilt_msg.Data = tilts(i);
pan_msg.Data = pans(i);
send(tilt_pub, tilt_msg);
send(pan_pub, pan_msg);
pause(2);
msg2 = receive(sub,10);
poses_base = [poses_base ; msg2.Pose(3)];
poses_pan = [poses_pan; msg2.Pose(7)];
poses_tilt = [poses_tilt; msg2.Pose(8)];
end