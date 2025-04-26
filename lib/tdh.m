function T = tdh(theta, d, a, alpha)
%TDH Generates a 4x4 transformation matrix from DH parameters.
% Inputs:
%   theta: rotation around z-axis (in radians)
%   d: translation along z-axis
%   a: translation along x-axis
%   alpha: rotation around x-axis (in radians)
%
% Outputs:
% T: a homogenious transformation matrix given by a
% rotz(theta)*t(d)*t(x)*rotx(alpha)

    T = [cos(theta),            -sin(theta)*cos(alpha),   sin(theta)*sin(alpha),   a*cos(theta);
         sin(theta),             cos(theta)*cos(alpha),  -cos(theta)*sin(alpha),   a*sin(theta);
         0,                      sin(alpha),              cos(alpha),              d;
         0,                      0,                       0,                       1];
end