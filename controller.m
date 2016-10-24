function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);

K_d = [ 2.4; 3.2; 60 ];
K_p = [ 32; 32; 800 ];

K_d_angle = [ 2.6; 2.6; 2.6 ];
K_p_angle = [ 160; 160; 160 ];

e_p = des_state.pos - state.pos;
e_v = des_state.vel - state.vel;

r_dotdot_des = des_state.acc + K_d .* e_v + K_p .* e_p;

F = params.mass * (params.gravity + r_dotdot_des(3));

if F < params.minF
    F = params.minF;
end
if F > params.maxF
    F = params.maxF;
end

phi_c = 1/params.gravity * (r_dotdot_des(1) * sin(des_state.yaw) - r_dotdot_des(2) * cos(des_state.yaw));

theta_c = 1/params.gravity * (r_dotdot_des(1) * cos(des_state.yaw) + r_dotdot_des(2) * sin(des_state.yaw));

rot_c = [ phi_c; theta_c; des_state.yaw ];

omega_c = [ 0; 0; des_state.yawdot ];

M = K_p_angle .* (rot_c - state.rot) + K_d_angle .* (omega_c - state.omega);

% =================== Your code ends here ===================

end
