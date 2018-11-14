function [T, robot_T] = ScaraFK(joint_angles, robot)

l_1 = robot.l_1;
l_2 = robot.l_2;

T01 = dhtf(0,0,0,joint_angles(1));  %  alp,a,d,the
T12 = dhtf(0, l_1,0, joint_angles(2));

l_2 = robot.l_2;
d3 = joint_angles(3);

T23 = dhtf(pi,l_2,-d3,0);

robot_T = {T01,T12,T23};
T = T01*T12*T23;