function adV = ad(V)
% ad represents the lie bracket of a twist acting as a spacial cross
% product
%
% 
% Inputs:
%       V: 6x1 twist 
% Output:
%       adV: 6x6 matrix that represents the cross product of the twist V
    omega = V(1:3);
    vel = V(4:6);
    adV = [skew(omega),zeros(3,3);skew(vel),skew(omega)];
end