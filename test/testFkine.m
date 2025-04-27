classdef testFkine < matlab.unittest.TestCase
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
            [s,m]=make_kinematics_model();
            my_robot= make_robot();

            testCase.S = s;
            testCase.S_body = adjoint(inv(m))*s;
            testCase.M = m;
            testCase.robot = my_robot;
        end
    end

    methods (TestMethodSetup)
        % Setup for each test
        
    end

    methods (Test)
        % Test methods

        function testHomeConfig(testCase)
            T_ref = double(testCase.robot.fkine(zeros(1,10)));
            T_fk = fkine(testCase.S,testCase.M,zeros(1,7),'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end
        function testSpaceJoint1(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint2(testCase)
            test_q = testCase.q(2,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint3(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint4(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint5(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint6(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testSpaceJoint7(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testAllSpace(testCase)
            test_q = ones(1,7);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S,testCase.M,test_q,'space');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end
        
        function testHomeConfigBody(testCase)
            T_ref = double(testCase.robot.fkine(zeros(1,10)));
            T_fk = fkine(testCase.S,testCase.M,zeros(1,7),'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint1(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint2(testCase)
            test_q = testCase.q(2,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint3(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint4(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint5(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint6(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testBodyJoint7(testCase)
            test_q = testCase.q(1,:);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end

        function testAllBody(testCase)
            test_q = ones(1,7);
            qr = testCase.compQ(test_q);
            T_ref = double(testCase.robot.fkine(qr));
            T_fk = fkine(testCase.S_body,testCase.M,test_q,'body');
            error = T_ref - T_fk;
            error = norm(error);
            testCase.assertTrue(error < 1e-4);
        end
    end
    methods (Access = private)
        function qr = compQ(testCase, q)
            qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)];
        end
    end

end