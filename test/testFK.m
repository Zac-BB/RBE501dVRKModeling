

[S,M] = make_kinematics_model()


q = [zeros(1,7);eye(7)]
for i = 1:size(q,1)
    T_abs
    
end


function qr = compParams(q)
    qr = [q(1) q(2) 0 -q(2) q(2) q(3) q(4) q(5) q(6) q(7)]
end