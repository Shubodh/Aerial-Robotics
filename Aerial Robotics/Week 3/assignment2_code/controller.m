function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% Approach: Before proceeding with u1 and u2, we need to evaluate phi_c
% as it is to be used in u2. Then, evaluation of u2 is 
% pretty straightforward by using the parameters/variables params, 
% des_state and state.


%1) K values:
Kp_phi = 1000; 
Kp_y = 20;
Kp_z = 100;

Kv_phi = 20;
Kv_y = 5;
Kv_z = 20;



%2)Extracting and storing the robot parameters
I_xx = params.Ixx;
m = params.mass;
g = params.gravity;

%3)Actual States and Desired States
y = state.pos(1);
z = state.pos(2);

y_dot = state.vel(1);
z_dot = state.vel(2);

phi = state.rot(1);
phi_dot = state.omega(1);


y_c = des_state.pos(1);
z_c = des_state.pos(2);

y_dot_c = des_state.vel(1);
z_dot_c = des_state.vel(2);

y_dot_dot_c = des_state.acc(1);
z_dot_dot_c = des_state.acc(2);


%4) Compute phi_c and phi_dot_c 

phi_c = -(1/g)*(y_dot_dot_c + Kv_y*(y_dot_c - y_dot) + Kp_y*(y_c - y));
%Assuming phidotc and phidotdotc to be zero
phi_dot_c = 0;
phi_dot_dot_c = 0;

%5) Calculating u1 and u2 after finding phi values
u1 = m*(g + z_dot_dot_c + Kv_z*(z_dot_c - z_dot) + Kp_z*(z_c - z));
u2 = I_xx*(phi_dot_dot_c + Kv_phi*(phi_dot_c - phi_dot) + Kp_phi*(phi_c - phi) );

end