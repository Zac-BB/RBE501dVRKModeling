function q = ikineUR5(x,S,M,q0)
% ikineUR5 determines the invere kinematics for the UR5 robot
%
% Inputs: 
%       x: 6x1 vector that represents the target pose in the space frame
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       q0: a 6x1 vector of joint displacenets that acts as an initial
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
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';
    state = false;
    while norm(x - currentPose) > .001
        Ja = jacoba(S,M,q);
        J = jacob0(S,q);
        if(norm(x - currentPose) < .1)
            state = true;
        end
        if state
            dq = pinv(J)*(x-currentPose);
        else
            dq = Ja'*(x(4:6,1) - p);
        end
        
        q = q + dq;
        T = fkine(S,M,q,"space");
        p = T(1:3,4);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
    end
end