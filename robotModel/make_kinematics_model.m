function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% a robot.
%
% Inputs: None
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% mm2m = 1/1000;
% l_2L3 = 40.09*mm2m;
% l_2H1 = 144.54*mm2m;
% l_2L2 = 516*mm2m;
% l_3 = 40.09*mm2m;
% l_RCC = 431.8*mm2m;
% l_c2 = -(-l_RCC + l_2H1);
% l_tool = 416.2*mm2m;
% l_p2y = 0.91*mm2m;
% prismaticLim = 400*mm2m;

% Screw Axes
S = [
   -0.0000    1.0000   -1.0000    1.0000         0   -0.0000   -1.0000   -0.0000   -0.0000;
   -1.0000   -0.0000    0.0000   -0.0000         0   -0.0000   -0.0000    1.0000   -0.0000;
    0.0000   -0.0000    0.0000   -0.0000         0   -1.0000    0.0000   -0.0000   -1.0000;
    0.0000   -0.0000    0.0000   -0.0000   -0.0000    0.4358   -0.0000    0.5598    0.4358;
   -0.0000   -0.0000   -0.1445    0.1445   -0.0000   -0.0000    0.5589   -0.0000   -0.0009;
   -0.0000   -0.0000    0.0401    0.4759   -1.0000   -0.0000   -0.4358   -0.0000   -0.0000;
];

M = [0,1,0,-0.00091;1,0,0,-0.4358;0,0,-1,-0.5598;0,0,0,1];


end