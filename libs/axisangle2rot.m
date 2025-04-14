function R = axisangle2rot(omega,theta)
    % your code here
    K = skew(omega);
    R = eye(3) + (sin(theta))*K + (1 - cos(theta))*(K*K);
end

