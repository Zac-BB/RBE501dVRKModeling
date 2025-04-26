function J_a = jacoba(S,M,q)    
% jacoba calculates the analitic jabobian 
%
% Inputs: 
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       robot too the end effector in the home configuration
%       q: a 6x1 vector of all of the joint displacements
%
% Output: 
%       J_a: 3xn matrix that represents the velocity of teh end effector
%
%
    T = fkine(S,M,q,"space");
    p = T(1:3,4);
    J = jacob0(S,q);
    J_v = J(4:6,:);
    J_w = J(1:3,:);
    
    J_a = J_v - skew(p)*J_w;
end