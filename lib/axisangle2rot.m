function R = axisangle2rot(omega,theta)
% axisangle2rot converts a axis of rotation ajd displacement too a rotation
% matrix
% 
% Inputs:
%       omega: angular axis of rotaion
%       theta: displacment about that axis
%
% Output:
%       R: rotation matrix that represents the rotaion about the axis
    K = skew(omega);
    R = eye(3) + (sin(theta))*K + (1 - cos(theta))*(K*K);
end