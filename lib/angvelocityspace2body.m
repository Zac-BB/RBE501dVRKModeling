function omega_b = angvelocityspace2body(omega_s,R)
% angvelocityspace2body converts a angular velocity in the space frame too
% the body frame
% 
% Inputs:
%       omega_s: 3x1 vector that is the angular axis of rotaion in the space frame
%       R: 3x3 rotation matrix from the base too the end effector
%
% Output:
%       omega_b: 3x1 vector that is the angular axis of rotation in the body frame
    R_dot = skew(omega_s)*R;
    bracket_omega_b = R'*R_dot;
    omega_b = [bracket_omega_b(3,2);bracket_omega_b(1,3);bracket_omega_b(2,1)];
end