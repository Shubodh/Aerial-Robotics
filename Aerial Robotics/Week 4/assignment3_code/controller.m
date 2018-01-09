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


g = params.gravity;
m = params.mass;
yaw = des_state.yaw;
yawdot = des_state.yawdot;

x_ddot_des = des_state.acc(1);
y_ddot_des = des_state.acc(2);
z_ddot_des = des_state.acc(3);

x_dot_des = des_state.vel(1);
y_dot_des = des_state.vel(2);
z_dot_des = des_state.vel(3);

x_dot = state.vel(1);
y_dot = state.vel(2);
z_dot = state.vel(3);

x_des = des_state.pos(1);
y_des = des_state.pos(2);
z_des = des_state.pos(3);

x = state.pos(1);
y = state.pos(2);
z = state.pos(3);

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

Kp = [200; 200; 200];
Kd = [40; 40; 40];

Kp_att = [100; 100; 100];
Kd_att = [2; 2; 2];

%STEP - 1)


%APPROACH: First of all (Step (2)) we write 3 equations for the
%components of acceleration of center of mass by setting the error
%exponentially go to zero, i.e. PD control equation. Next, using Newton's
%Equations of Motion, from the first two component equations, we get
%phi_des and theta_des (Step (3)). Now, from the third component equation
%remaining, we get the u1 value (Step (4)). To find out the value of u2, we
%apply the attitude control loop equation (near nominal hover state)(Step
%(5))similar to step 2. Thus, u1 and u2 are found out.
%
% STEP - 2) Finding rddot_des



x_ddot = x_ddot_des + Kd(1)*(x_dot_des - x_dot) + Kp(1)*(x_des - x);
y_ddot = y_ddot_des + Kd(2)*(y_dot_des - y_dot) + Kp(2)*(y_des - y);
z_ddot = z_ddot_des + Kd(3)*(z_dot_des - z_dot) + Kp(3)*(z_des - z);


% STEP - 3) Calculating phi_des and theta_des from Newton's equations of
% motion: NOTE: cos of yaw angle is 1 and sin of yaw is the angle itself as
% it is considered near hover position
phi_des   	= (x_ddot * yaw - y_ddot *  1 )/g ;
theta_des 	= (x_ddot * 1   + y_ddot * yaw)/g ;

% STEP - 4) Finding u1
u1 = m * (g + z_ddot);


% STEP - 5) Finding u2, here omega_des roll and pitch is 0 for hovering.
omega_des_roll = 0;
omega_des_pitch = 0;
u2 =[Kp_att(1) * (phi_des - phi) + Kd_att(1)  * (omega_des_roll - p);
	 Kp_att(2) * (theta_des -theta) + Kd_att(2) * (omega_des_pitch - q);
	 Kp_att(3) * (yaw - psi) + Kd_att(3) * (yawdot - r)];



% Thrust
F = u1;

% Moment
M = u2;


end
