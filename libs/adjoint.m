function AdT = adjoint(T)
    % your code here
    R = T(1:3,1:3);
    p = T(1:3,4);
    AdT = [R zeros(3,3); skew(p)*R R];
end
