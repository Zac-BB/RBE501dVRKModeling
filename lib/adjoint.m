function AdT = adjoint(T)
% adjoint represents the transformation of twists too frame T
% 
% Inputs:
%       T: 4x4 homogenious transformation matrix from the frame tha the twist is represented in too the desired framer
%
% Output:
%       AdT: 6x6 matrix that represents the transformation of a twist to
%       expressed in frame T
    R = T(1:3,1:3);
    p = T(1:3,4);
    AdT = [R zeros(3,3); skew(p)*R R];
end
