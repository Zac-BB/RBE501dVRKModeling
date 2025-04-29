classdef testJacob < matlab.unittest.TestCase
    properties
        q = [eye(7)]
        S
        M
        S_body
        robot
    end

    methods (TestClassSetup)
        % Shared setup for the entire test class
        function createModels(testCase)
            [s,m ]=make_kinematics_model();
            my_robot= make_robot();
            
            testCase.S = s;
            testCase.S_body = adjoint(m)*s;
            testCase.M = m;
            testCase.robot = my_robot;
        end
    end

    methods (TestMethodSetup)
        % Setup for each test
        
    end

    methods (Test)
        % Test methods

        function testSpaceHomeConfig(testCase)
            test_q = zeros(1,7);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end
        function testSpaceJoint1(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
            
        end

        function testSpaceJoint2(testCase)
            test_q = testCase.q(2,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testSpaceJoint3(testCase)
            test_q = testCase.q(3,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testSpaceJoint4(testCase)
            test_q = testCase.q(4,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testSpaceJoint5(testCase)
            test_q = testCase.q(5,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testSpaceJoint6(testCase)
            test_q = testCase.q(6,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testSpaceJoint7(testCase)
            test_q = testCase.q(7,:);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end

        function testAllSpace(testCase)
            test_q = ones(1,7);
            qr = testCase.compQ(test_q);
            J = jacob0(testCase.S,test_q);
            T = fkine(testCase.S,testCase.M,test_q,'space');
            Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end


        function testBodyHomeConfig(testCase)
            test_q = zeros(1,7);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacob0(qr));
            syms dq1 dq2 dq3 dq4 dq5 dq6 dq7
            syms_refJ = refJ*[dq1 dq2 0 -dq2 dq2 dq3 dq4 dq5 dq6 dq7]';
            syms_J = Jcoords*[dq1 dq2 dq3 dq4 dq5 dq6 dq7]';
            testCase.assertTrue(all(double(subs(syms_refJ - syms_J,[dq1 dq2 dq3 dq4 dq5 dq6 dq7],ones(1,7))) < 1e-4));
        end
        function testBodyJoint1(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
            
        end

        function testBodyJoint2(testCase)
            test_q = testCase.q(2,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testBodyJoint3(testCase)
            test_q = testCase.q(3,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testBodyJoint4(testCase)
            test_q = testCase.q(4,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testBodyJoint5(testCase)
            test_q = testCase.q(5,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testBodyJoint6(testCase)
            test_q = testCase.q(6,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testBodyJoint7(testCase)
            test_q = testCase.q(7,:);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end

        function testAllBody(testCase)
            test_q = ones(1,7);
            qr = testCase.compQ(test_q);
            J_b = jacobe(testCase.S,testCase.M,test_q);
            Jcoords = [J_b(4:6,:); J_b(1:3,:)];
            refJ = double(testCase.robot.jacobe(qr));
            syms_refJ = refJ*testCase.compQ(ones(1,7))';
            syms_J = Jcoords*ones(1,7)';
            testCase.assertTrue(all(double(syms_refJ - syms_J) < 1e-4));
        end
    end
    methods (Access = private)
        function qr = compQ(testCase, q)
            qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)];
        end
    end

end