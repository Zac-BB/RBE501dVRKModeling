function traj = make_trajectory(type, params)
% make_trajectory generates a trajectory from q(1) too q(2)
%
% type - a string indicating the type of trajectory that you want to generate.
%                Acceptable values: {'cubic' | 'quintic'}
%
% params - a structure containing the prescribed trajectory parameters
%   params.t  - 2-vector prescribing the initial and final time
%   params.dt - desired time step
%   params.q - 2-vector prescribing the starting and final positions
%   params.v - 2-vector describing the starting and final velocities
%   params.a - 2-vector describing the starting and final accelerations (only for quintic polynomials)
% traj - a structure containing the trajectory
%   traj.t - n-vector representing time
%   traj.q - n-vector representing position over time
%   traj.v - n-vector representing velocity over time
%   traj.a - n-vector representing acceleration over time


    if(strcmp(type,'cubic'))
        a = inv([1, params.t(1), params.t(1)^2, params.t(1)^3;0, 1, 2*params.t(1), 3*params.t(1)^2;1, params.t(2), params.t(2)^2, params.t(2)^3;0, 1, 2*params.t(2), 3*params.t(2)^2])*[params.q(1);params.v(1);params.q(2);params.v(2)];
        traj.t = params.t(1):params.params.time_step:params.t(2);
        t = traj.t;
        one = ones(1,size(traj.t,2));
        traj.q = a'*[one;t;t .* t;t .* t .* t];
        traj.v = a'*[0*one;one;2*t;3*(t .* t)];
        traj.a = a'*[0*one;0*one;2*one;6*t];
    elseif(strcmp(type,'quintic'))
        a = inv([
            1, params.t(1), params.t(1)^2, params.t(1)^3, params.t(1)^4, params.t(1)^4;
            0, 1, 2*params.t(1), 3*params.t(1)^2, 4*params.t(1)^3, 5*params.t(1)^4;
            0, 0, 2, 6*params.t(1), 12*params.t(1)^2, 20*params.t(1)^3;
            1, params.t(2), params.t(2)^2, params.t(2)^3, params.t(2)^4, params.t(2)^5;
            0, 1, 2*params.t(2), 3*params.t(2)^2,4*params.t(2)^3,5*params.t(2)^4;
            0, 0, 2, 6*params.t(2), 12*params.t(2)^2, 20*params.t(2)^3;] ...
            )*[params.q(1);params.v(1);params.a(1);params.q(2);params.v(2);params.a(2)];
        traj.t = params.t(1):params.time_step:params.t(2);
        t = traj.t;
        one = ones(1,size(traj.t,2));
        traj.q = a'*[one;t;t.^2;t.^3;t.^4;t.^5];
        traj.v = a'*[0*one;one;2*t;3*t.^2;4*t.^3;5*t.^4];
        traj.a = a'*[0*one;0*one;2*one;6*t;12*t.^2;20*t.^3];
    end

end