function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% u = 0;
params = sys_params;
% u = params.mass*params.gravity;

k_p = 140;
k_v = 20;
e = s_des - s;
% Setting zdes2 to zero
zdes_2 = 0;
u = params.mass * (zdes_2 + k_p * e(1) + k_v * e(2) + params.gravity) ;

% FILL IN YOUR CODE HERE


end

