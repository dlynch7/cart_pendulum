%% main.m
%
% Description:
%   Application entry point.
%
% Inputs: none
%
% Outputs: none
%
% Notes:

function main

%% Initialize environment
clear;
close all;
clc;

init_env();

%% Initialize parameters
params = init_params;

%% Visualize the robot in its initial state
x_IC = [params.sim.ICs.x_cart;
     params.sim.ICs.theta_pend;
     params.sim.ICs.dx_cart;
     params.sim.ICs.dtheta_pend];
 
plot_robot(x_IC(1:2),params,'new_fig',false);

%% Simulate the robot forward in time with no control input
tspan_passive = 0:params.sim.dt:5;
[tsim_passive, xsim_passive] = ode45(@(t,x) robot_dynamics(...
    t,x,0,params,'controller','passive'),...
    tspan_passive, x_IC');

% tranpose xsim_passive so that it is 4xN (N = number of timesteps):
xsim_passive = xsim_passive'; % required by animate_robot.m

% figure;
% subplot(2,1,1), plot(tsim_passive,xsim_passive(1,:),'b-',...
%                      tsim_passive,xsim_passive(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_passive,xsim_passive(3,:),'b:',...
%                      tsim_passive,xsim_passive(4,:),'r:','LineWidth',2);


pause(1); % helps prevent animation from showing up on the wrong figure
% animate_robot(xsim_passive(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',false);
fprintf('Done passive simulation.\n');

%% Control the unstable equilibrium with LQR
A = upright_state_matrix(params);
B = upright_input_matrix(params);

% numerical verify the rank of the controllability matrix:
Co = [B, A*B, (A^2)*B, (A^3)*B];
fprintf('rank(Co) = %d.\n',rank(Co));

% control design: weights Q and R:
Q = diag([5000,100,1,1]);    % weight on regulation error
R = 1;                  % weight on control effort

% compute and display optimal feedback gain matrix K:
K = lqr(A,B,Q,R);
buf = '';
for i = 1:size(K,2)
    buf = [buf,'%5.3f '];
end
buf = [buf,'\n'];
fprintf('LQR: K = \n');
fprintf(buf,K');

% we could ask what are the eigenvalues of the closed-loop system:
eig(A - B*K)

% add K to our struct "params":
params.control.inverted.K = K;

% Simulate the robot under this controller:
tspan_stabilize = 0:params.sim.dt:5;
[tsim_stabilize, xsim_stabilize] = ode45(@(t,x) robot_dynamics(...
    t,x,0,params,'controller','stabilize'),...
    tspan_stabilize, x_IC');

% tranpose xsim_passive so that it is 4xN (N = number of timesteps):
xsim_stabilize = xsim_stabilize'; % required by animate_robot.m

% figure;
% subplot(2,1,1), plot(tsim_stabilize,xsim_stabilize(1,:),'b-',...
%                      tsim_stabilize,xsim_stabilize(2,:),'r-','LineWidth',2);
% subplot(2,1,2), plot(tsim_stabilize,xsim_stabilize(3,:),'b:',...
%                      tsim_stabilize,xsim_stabilize(4,:),'r:','LineWidth',2);
% pause(1); % helps prevent animation from showing up on the wrong figure


% animate_robot(xsim_stabilize(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',false);
fprintf('Done passive simulation.\n');

%% Find a fixed-time swingup trajectory by solving a TPBVP

% % Set up the fixed-time TPBVP:
% opts_fixed = bvpset('RelTol',0.1,'AbsTol',0.1*ones(1,8),'Stats','on',...
%     'FJacobian',@(t,z) grad_tpbvp_ode(t,z,params));
% 
% % fixed time:
% tmesh_fixed = linspace(0,2,params.control.swingup.TPBVP.Nmesh);
% 
% % initial guess: configuration changes linearly in time, roughly constant
% % velocities, costate ???
% % initial guess (robot state)
% z_fixed_init(1,:) = linspace(x_IC(1),...
%                        params.control.inverted.x_eq(1),...
%                        params.control.swingup.TPBVP.Nmesh);
% z_fixed_init(2,:) = linspace(x_IC(2),...
%                        params.control.inverted.x_eq(2),...
%                        params.control.swingup.TPBVP.Nmesh);
% z_fixed_init(3,:) = [x_IC(3),...
%     diff(z_fixed_init(1,1:params.control.swingup.TPBVP.Nmesh-1)),...
%     params.control.inverted.x_eq(3)];
% z_fixed_init(4,:) = [x_IC(4),...
%     diff(z_fixed_init(2,1:params.control.swingup.TPBVP.Nmesh-1)),...
%     params.control.inverted.x_eq(4)];
% 
% % initial guess (robot costate):
% z_fixed_init(5,:) = zeros(1,params.control.swingup.TPBVP.Nmesh);
% z_fixed_init(6,:) = zeros(1,params.control.swingup.TPBVP.Nmesh);
% z_fixed_init(7,:) = zeros(1,params.control.swingup.TPBVP.Nmesh);
% z_fixed_init(8,:) = zeros(1,params.control.swingup.TPBVP.Nmesh);
% 
% solinit_fixed.x = tmesh_fixed;
% solinit_fixed.y = z_fixed_init;
% 
% % Solve the fixed-time TPBVP:
% sol4c_fixed = bvp4c(@(t,z) tpbvp_ode(t,z,params),...
%     @(z0,zT) tpbvp_bc(z0,zT,params),...
%     solinit_fixed, opts_fixed);
% % sol5c_fixed = bvp5c(@(t,z) tpbvp_ode(t,z,params),...
% %     @(z0,zT) tpbvp_bc(z0,zT,params),...
% %     solinit_fixed, opts_fixed);
% 
% tsim_swingup_fixed = sol4c_fixed.x;
% xsim_swingup_fixed = sol4c_fixed.y;
% 
% figure;
% subplot(2,1,1), plot(tsim_swingup_fixed,xsim_swingup_fixed(1,:),'b-',...
%                      tsim_swingup_fixed,xsim_swingup_fixed(2,:),'r-',...
%                      'LineWidth',2);
% subplot(2,1,2), plot(tsim_swingup_fixed,xsim_swingup_fixed(3,:),'b:',...
%                      tsim_swingup_fixed,xsim_swingup_fixed(4,:),'r:',...
%                      'LineWidth',2);
% pause(1); % helps prevent animation from showing up on the wrong figure
% 
% animate_robot(xsim_swingup_fixed(1:2,:),params,'trace_cart_com',true,...
%     'trace_pend_com',true,'trace_pend_tip',true,'video',false);

%% Find a free-time swingup trajectory by solving an augmented-state TPBVP:

% Set up the free-time TPBVP:
% opts_free = bvpset('RelTol',0.1,'AbsTol',0.1*ones(1,9),'Stats','on',...
%     'FJacobian',@(t,z) grad_tpbvp_ode(t,z,params));
opts_free = bvpset('RelTol',0.1,'AbsTol',0.1*ones(1,9),'Stats','on');

% free time is scaled to dimensionless fixed time:
tmesh_free = linspace(0,1,params.control.swingup.TPBVP.Nmesh);

% initial guess: configuration changes linearly in time, roughly constant
% velocities, costate ???
% initial guess (robot state)
z_free_init(1,:) = linspace(x_IC(1),...
                       params.control.inverted.x_eq(1),...
                       params.control.swingup.TPBVP.Nmesh);
z_free_init(2,:) = linspace(x_IC(2),...
                       params.control.inverted.x_eq(2),...
                       params.control.swingup.TPBVP.Nmesh);
z_free_init(3,:) = [x_IC(3),...
    diff(z_free_init(1,1:params.control.swingup.TPBVP.Nmesh-1)),...
    params.control.inverted.x_eq(3)];
z_free_init(4,:) = [x_IC(4),...
    diff(z_free_init(2,1:params.control.swingup.TPBVP.Nmesh-1)),...
    params.control.inverted.x_eq(4)];

% initial guess (robot costate):
z_free_init(5,:) = ones(1,params.control.swingup.TPBVP.Nmesh);
z_free_init(6,:) = ones(1,params.control.swingup.TPBVP.Nmesh);
z_free_init(7,:) = ones(1,params.control.swingup.TPBVP.Nmesh);
z_free_init(8,:) = ones(1,params.control.swingup.TPBVP.Nmesh);

% initial guess (terminal time):
z_free_init(9,:) = 2*ones(1,params.control.swingup.TPBVP.Nmesh);

solinit_free.x = tmesh_free;
solinit_free.y = z_free_init;

% Solve the free-time TPBVP:
sol4c_free = bvp4c(@(t,z) tpbvp_ode(t,z,params),...
    @(z0,zT) tpbvp_bc(z0,zT,params),...
    solinit_free, opts_free);
% sol5c_free = bvp5c(@(t,z) tpbvp_ode(t,z,params),...
%     @(z0,zT) tpbvp_bc(z0,zT,params),...
%     solinit_free, opts_free);

tsim_swingup_free = sol4c_free.x;
xsim_swingup_free = sol4c_free.y;

figure;
subplot(2,1,1), plot(tsim_swingup_free,xsim_swingup_free(1,:),'b-',...
                     tsim_swingup_free,xsim_swingup_free(2,:),'r-',...
                     'LineWidth',2);
subplot(2,1,2), plot(tsim_swingup_free,xsim_swingup_free(3,:),'b:',...
                     tsim_swingup_free,xsim_swingup_free(4,:),'r:',...
                     'LineWidth',2);
pause(1); % helps prevent animation from showing up on the wrong figure

animate_robot(xsim_swingup_free(1:2,:),params,'trace_cart_com',true,...
    'trace_pend_com',true,'trace_pend_tip',true,'video',true);
end
