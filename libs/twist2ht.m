function T = twist2ht(S,theta)
    omega = S(1:3);
    v = S(4:6);
    R = axisangle2rot(omega,theta);
    K = skew(omega);
    p = (eye(3)*theta + (1-cos(theta))*K + (theta-sin(theta))*K*K)*v;
    T = [R p; 0 0 0 1]
end
