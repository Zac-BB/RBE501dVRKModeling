clear;close all
robot = make_robot()

q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
q7 = 0;

q = [q1 q2 0 -q2 q2 q3 q4 q5 q6 q7]
robot.teach(q)