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
x = [params.sim.ICs.x_cart;
     params.sim.ICs.theta_pend;
     params.sim.ICs.dx_cart;
     params.sim.ICs.dtheta_pend];
 
plot_robot(x(1:2),params,...
    'new_fig',false,...
    'trace_cart_com',true,...
    'trace_pend_com',true,...
    'trace_pend_tip',true);
plot_robot(0.5.*x(1:2),params,...
    'new_fig',false,...
    'trace_cart_com',true,...
    'trace_pend_com',true,...
    'trace_pend_tip',true);

%% Simulate the robot forward in time with no control input



end