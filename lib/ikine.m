function q = ikine(x,S,M,q0)
% ikine determines the invere kinematics for a robot it does not acount
% for end effector orientation
%
% Inputs: 
%       x: 3x1 vector that represents the target pose in the space frame
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       q0: a nx1 vector of joint displacenets that acts as an initial
%       guess
%
% Output: 
%       q: nx1 vector of the joint displacments to achive the desired
%       position
%
%
    q = q0;
    T = fkine(S,M,q,"space");
    p = T(1:3,4);
    while norm(x - p) > .001
        Ja = jacoba(S,M,q);
        dq = Ja'*(x - p);
        q = q + dq;
        T = fkine(S,M,q,"space");
        p = T(1:3,4);
        
    end
end