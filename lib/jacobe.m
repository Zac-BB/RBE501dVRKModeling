function J_b = jacobe(S,M,q)  
% jacobe calculates the jabobian in the end effector frame
%
% Inputs: 
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       robot too the end effector in the home configuration
%       q: a 6x1 vector of all of the joint displacements
%
% Output: 
%       J_b: 6xn matrix that is the body frame jacobian
%
%
    T = fkine(S,M,q,"space");
    Ad = adjoint(inv(T));
    J_s = jacob0(S,q);
    J_b = Ad * J_s;

end