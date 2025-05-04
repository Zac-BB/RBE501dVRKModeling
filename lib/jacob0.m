function J = jacob0(S,q)
% jacob0 calculates the jabobian in the space frame
%
% Inputs: 
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       robot too the end effector in the home configuration
%       q: a 6x1 vector of all of the joint displacements
%
% Output: 
%       J: 6xn matrix representing the space fram jacobian
%
%
    T = eye(4);
    q = [q(1) q(2) q(2) q(2) q(3) q(4) q(5) q(6) q(7)];
    for i = 1:size(S,2)
        Si = S(:,i);
        J(:,i) = adjoint(T)*Si;
        T = T * twist2ht(Si,q(i));
        
    end
    J = [J(:,1),J(:,2)+J(:,3)+J(:,4),J(:,5),J(:,6),J(:,7),J(:,8),J(:,9)];

end
