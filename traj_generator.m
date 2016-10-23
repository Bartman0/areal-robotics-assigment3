function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

    persistent waypoints0 traj_time d0 ax ay az
    if nargin > 2
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;

        ax = coeffs(waypoints0(1,:))';
        ay = coeffs(waypoints0(2,:))';
        az = coeffs(waypoints0(3,:))';
    else
        if(t > traj_time(end))
            t = traj_time(end);
        end
        t_index = find(traj_time >= t,1) - 1;

        if(t == 0)
            desired_state.pos = waypoints0(:,1);
            desired_state.vel = 0*waypoints0(:,1);
            desired_state.acc = 0*waypoints0(:,1);
        else
            t_scale = (t-traj_time(t_index))/d0(t_index);
            index = (t_index - 1) * 8 + 1 : t_index * 8;
            p = polynom(8, 0, t_scale)';
            desired_state.pos = [ax(index) * p; ay(index) * p; ax(index) * p];
        
            pdot = polynom(8, 1, t_scale)';
            desired_state.vel = [ax(index) * pdot; ay(index) * pdot; az(index) * pdot].*(1/d0(t_index));

            pddot = polynom(8, 2, t_scale)';
            desired_state.acc = [ax(index) * pddot; ay(index) * pddot; az(index) * pddot].*(1/d0(t_index)^2);
        end
    %     desired_state.vel = zeros(3,1);
    %     desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end

end

function [C]=coeffs(w)
    [n_coords, n] = size(w);
    n = n - 1;
    
    A = zeros(8*n,8*n);
    b = zeros(8*n, 1);

    for i = 1:n
        b(i) = w(i);
        b(n+i) = w(i+1);
    end
    
    y_shift = 0;
    for i = 1:n
        x_shift = 8*(i-1);
        A(y_shift+i,x_shift+1:x_shift+8) = polynom(8,0,0);
    end
    
    y_shift = n;
    for i = 1:n
        x_shift = 8*(i-1);
        A(y_shift+i,x_shift+1:x_shift+8) = polynom(8,0,1);
    end
    
    y_shift = 2*n;
    for k = 1:3
        A(y_shift+k,1:8) = polynom(8, k, 0);
    end
    
    y_shift = 2*n + 3;
    for k = 1:3
        A(y_shift+k,end-7:end) = polynom(8, k, 1);
    end

    for i = 1:n-1
        x_shift = 8*(i-1);
        for k = 1:6
            y_shift = 2*n+6 + 6*(i-1);
            A(y_shift+k, x_shift+1:x_shift+16) = [polynom(8, k, 1), -1 * polynom(8, k, 0)];
        end
    end
   
    C = inv(A) * b;
end
