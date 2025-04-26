function V_b = twistspace2body(V_s,T)
% twistspace2body converts space frame twists to body frame twists
%
% Inputs: 
%       V_S: a 6xn matrix of the space frame screw axies
%       T: is the transform from 
%
% Output: 
%       V_b: a 6xn matrix of the body frame screw axies
%
%
    R = T(1:3,1:3);
    P = T(1:3,4);
    omega_s = V_s(1:3,1);
    v_s = V_s(4:6,1);
    omega_b = angvelocityspace2body(omega_s,R);
    p_dot = v_s + skew(omega_s)*P;
    v_b = R'*p_dot;
    V_b = [omega_b;v_b];
    
end