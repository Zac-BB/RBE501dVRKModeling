function robot = make_robot()
%MAKE_ROBOT Creates the kinematic structure of the robot used in homework 4, problem 1.
%
%   This is a factory function that creates the robot used in the homework.
%
%   Inputs: none
%
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: Zac Serocki <lfichera@wpi.edu>
%   Last modified: 04/23/2025

%% Create the manipulator

mm2m = 1/1000;
l_2L3 = 40.09*mm2m;
l_2H1 = 144.54*mm2m;
l_2L2 = 516*mm2m;
l_3 = 40.09*mm2m;
l_RCC = 431.8*mm2m;
l_c2 = -(-l_RCC + l_2H1);
l_tool = 416.2*mm2m;
l_p2y = 0.91*mm2m;
prismaticLim = 400*mm2m;
robot = SerialLink([RevoluteMDH('a', 0, 'alpha', pi/2, 'd', 0, 'offset', pi/2), ...%1
                    RevoluteMDH('a', 0, 'alpha', pi/2, 'd', 0, 'offset', -pi/2), ... %2
                    RevoluteMDH('a', l_2L3, 'alpha', 0, 'd', 0, 'offset', pi/2), ...% is ZERO 2'
                    RevoluteMDH('a', l_2H1, 'alpha', 0, 'd', 0, 'offset', pi/2), ...% is q2 2''
                    RevoluteMDH('a', l_2L2, 'alpha', 0, 'd', 0, 'offset', pi), ...% is q2 2''''
                    PrismaticMDH('a', l_3, 'alpha', pi/2, 'theta', 0, 'offset', l_c2,'qlim',prismaticLim), ...
                    RevoluteMDH('a', 0, 'alpha', 0, 'd', l_tool, 'offset', 0), ...
                    RevoluteMDH('a', 0, 'alpha', pi/2, 'd', 0, 'offset', pi/2), ...
                    RevoluteMDH('a', l_p2y, 'alpha', pi/2, 'd', 0, 'offset', pi/2), ...
                    RevoluteMDH('a', l_p2y, 'alpha', pi/2, 'd', 0, 'offset', pi/2), ...
                    ], ...
                    'name', 'PRM robot');


end
                    % RevoluteMDH('a', l_2L1, 'alpha', 0, 'd', 0, 'offset', pi), ...% is q2 
