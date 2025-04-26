function T = fkine(S,M,q,frame)
% fkine determines the forward kinematics for a robot
%
% Inputs: 
%       S: a 6xn matrix of the space frame screw axies
%       M: a 4x4 homogenious transformation matrix from the base of the
%       q: a 6x1 vector of joint displacenets 
%       frame: a string representing the frame that the S and M matricies
%       are in 
%       Acceptable values: {'space' | 'body'}
%
% Output: 
%       q: nx1 vector of the joint displacments to achive the desired
%       position
%
%
    if(strcmp(frame,'body'))
        T = M;
        for i = 1:size(S,2)
            Si = S(:,i);
            qi = q(i);
            T =  T* twist2ht(Si,qi);
        end
    elseif(strcmp(frame,'space'))
        T = eye(4);
        for i = 1:size(S,2)
            Si = S(:,i);
            qi = q(i);
            T =  T* twist2ht(Si,qi);
        end
        T = T* M;
    else
        T = eye(4);
    end
    
end