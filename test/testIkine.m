[S,M] = make_kinematics_model()

nTests = 10;
plotOn = true;

%% Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

zeros_Q = zeros(7, 1);

if plotOn
    robot.teach(compParams(zeros_Q));
end

% Joint limits
qlim = [-pi/4  pi/4;  % q(1)
        -pi/4  pi/4;  % q(2)
        -pi/4  pi/4;  % q(3)
        -pi/4  pi/4;  % q(4)
        -pi/4  pi/4;  % q(5)
        -pi/4  pi/4;
        -pi/4  pi/4]; % q(6)

% Generate the test configurations
% This linspace covers full width of workspace, but occasionally creates
% target adjacent to singularity, so it is not active
q = [linspace(qlim(1, 1), qlim(1, 2),nTests);
     linspace(qlim(2, 1), qlim(2, 2),nTests);
     linspace(qlim(3, 1), qlim(3, 2), nTests);
     linspace(qlim(4, 1), qlim(4, 2),nTests);
     linspace(qlim(5, 1), qlim(5, 2),nTests);
     linspace(qlim(6, 1), qlim(6, 2), nTests);
     linspace(qlim(7, 1), qlim(7, 2), nTests)];

for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));

    % Generate the robot's pose
    qr = compParams(q(:,ii));
    T = fkine(S,M,q(:,ii)', 'space');
    targetPose = T(1:3, 4);

    
    % Inverse Kinematics
    found_q = ikine(targetPose, S, M, zeros_Q);

    if plotOn
        try
            robot.teach(compParams(found_q));
            drawnow;
        catch e
            continue;
        end
    end
end

fprintf('\nTest passed successfully.\n');


function qr = compParams(q)
    qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)];
end