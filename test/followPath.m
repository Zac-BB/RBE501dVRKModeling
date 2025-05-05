% Robot
[S,M] = make_kinematics_model();

% Generate Path
nPts = 100;
fprintf('Generating task space path... ');
phi = linspace(-1/3*pi, 1/3*pi, nPts);
rx = 1;
ry = 0.3;
rz = 0.8;
x = rx .* sin(phi);
y = ry  .* sin(phi) - 0.6;
z = -rz .* cos(phi);
path = [x; y; z];

fprintf('Calculating the Inverse Kinematics... ');

zeros_Q = zeros(7, 1);

% robot.plot(compParams(zeros_Q));
hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

curr_q = zeros_Q;
q_records = zeros(10, nPts);
for i = 1:nPts
    curr_q = ikine(path(:, i), S, M, curr_q);
    q_records(:, i) = compParams(curr_q);

end

%% Animate the robot
title('Inverse Kinematics');
robot.plot(q_records','trail',{'r', 'LineWidth', 2}, 'movie', 'RBE-501-2023-Final_daVinciArch.avi');
fprintf('Done.\n');

function qr = compParams(q)
    qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)];
end