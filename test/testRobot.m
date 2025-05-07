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

T = robot.fkine(q)
[S_fake,M] = make_kinematics_model()


q = [eye(10)]
S = zeros(6,10)
q(3,3) = 0
for i = 1:size(q,1)
    T=robot.fkine(q(i,:))
    pure_T = double(T)*inv(M)
    Si = MatrixLog6(pure_T)
    Si = [Si(3,2) Si(1,3) Si(2,1) Si(1:3,4)']'
    S(:,i) = Si
end



function qr = compParams(q)
    qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)]
end