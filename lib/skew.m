function K = skew(v)
% skew converts a twist too a skew symetric matrix
%
% Inputs: 
%       v: a six dimentional vector
%
% Output: 
%       K: a 3x3 matrix that is the vetor skewed 
%
%
    K = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end