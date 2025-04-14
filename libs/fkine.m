function T = fkine(S,M,q)
    T = eye(4);
    for i = 1:size(S,2)
        Si = S(:,i);
        qi = q(i);
        T =  T* twist2ht(Si,qi);
    end
    T = T* M;
end