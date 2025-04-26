function [tau,V,Vdot] = rne(params)
%% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%
% Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.jointAcc - n-dimensional column vector of joint accelerations
%           params.Ftip - 6-dimensional column vector representing the
%           wrench applied at the tip
%
% Output: tau  - n-dimensional column vector of generalized joint forces
%         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%
% Forward iterations
n = size(params.S,2);
V = zeros(6,n+1);
Vdot = zeros(6,n+1);
tau = zeros(n,1);
A = zeros(6,n);
M = eye(4);
for i = 1:n
    M = M*params.M(:,:,i);
    A(:,i)=adjoint(inv(M))*params.S(:,i);
end



params.g = params.g(:);
Vd0 = [zeros(3,1); -params.g];
V = zeros(6, n);
Vdot = zeros(6, n);
for i = 1:n 
      T = twist2ht(-A(:, i), params.jointPos(i)) * inv(params.M(:, :, i));
      if i == 1
          V(:, i) = A(:, i) * params.jointVel(i) + adjoint(T) * zeros(6, 1);
          Vdot(:, i) = A(:, i) * params.jointAcc(i) + adjoint(T) * Vd0 + ad(V(:, i)) * (A(:, i))*params.jointVel(i);
      
      else
        V(:, i) = A(:, i) * params.jointVel(i) + adjoint(T) * V(:, i-1);
        Vdot(:, i) = A(:, i) * params.jointAcc(i) + adjoint(T) * Vdot(:, i-1) + ad(V(:, i)) * (A(:, i))*params.jointVel(i);
      end
 end
 
 V = [zeros(6, 1) V];
 Vdot = [Vd0 Vdot];

F = zeros(6,n+1);
F(:,n+1) = params.Ftip(:);

 for i = n:-1:1
     if i == n
         T = inv(params.M(:,:,i+1));
     else
         T = twist2ht(-A(:,i+1), params.jointPos(i+1))*inv(params.M(:,:,i+1));
     end
     F(:,i) = params.G(:,:,i)*Vdot(:,i+1) - (ad(V(:,i+1))')*params.G(:,:,i)*V(:,i+1) + (adjoint(T)')*F(:,i+1);
     tau(i) = F(:,i)'*A(:,i);

 end


end
