function T = twist2ht(S,theta)
% twist2ht converts space frame twists and angular displacement to a
% homogenious transformation matrix using Rodrigues' rotation formula
%
% Inputs: 
%       S: a 6xn matrix of the space frame screw axies
%       theta: a nx1 vector of angular displacemnet of the joints
%
% Output: 
%       T: a homogenious transformation matrix 
%
%
    omega = S(1:3);
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    K = skew(omega);
    p = (eye(3)*theta + (1-cos(theta))*K + (theta-sin(theta))*K*K)*v;
    T = [R p; 0 0 0 1];
end