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
tspan = 0:params.sim.dt:5;
[tsim, xsim] = ode45(@(t,x) robot_dynamics(t,x,0,params), tspan, x_IC');
xsim = xsim'; % tranpose so that xsim is 4xN (N = number of timesteps)

figure;
subplot(2,1,1), plot(tsim,xsim(1,:),'b-',tsim,xsim(2,:),'r-','LineWidth',2);
subplot(2,1,2), plot(tsim,xsim(3,:),'b:',tsim,xsim(4,:),'r:','LineWidth',2);

pause(1);

animate_robot(xsim(1:2,:),params,'trace_cart_com',true,...
    'trace_pend_com',true,'trace_pend_tip',true,'video',true);
fprintf('Done!\n');


end