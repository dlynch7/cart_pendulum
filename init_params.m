%% init_params.m
%
% Description:
%   Initializes the values of many parameters, such as parameters in the
%   system dynamics, parameters that relate to simulating the system
%   forward in time, and parametes that relate to visualization/animation.
%   
% Inputs:
%   none

% Outputs:
%   params: a struct with many elements

function params = init_params
    % parameters that appear in the dynamics:
    params.model.dyn.cart.m = 1;    % mass of the cart
    params.model.dyn.pend.m = 1;    % mass of the pendulum
    params.model.dyn.pend.I = 0.001;% moment of inertia of the pendulum
    params.model.dyn.pend.r_com = 0.5; % radial loc. of pendulum CoM
    params.model.dyn.g = 9.81;      % acceleration due to gravity
    params.model.dyn.b1 = 0.0;      % damping between cart and track
    params.model.dyn.b2 = 0.05;     % damping between pendulum and cart
    
    % parameters that help with visualizing the robot:
    params.model.geom.cart.w = 0.5; % width of the cart
    params.model.geom.cart.h = 0.5; % height of the cart
    params.model.geom.pend.l = 1;   % length of the pendulum
    params.model.geom.pend.w = 0.1; % width of the pendulum
    
    params.viz.colors.cart = [0.5 0.5 0.5];
    params.viz.colors.pend = [0.25 0.25 0.25];
    params.viz.colors.pend_com = [0.75 0.75 0.75];
    
    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.x_cart = -0.5;      % initial cart position
    params.sim.ICs.theta_pend = -pi/2;  % initial pendulum angle
    params.sim.ICs.dx_cart = 0;     % initial cart velocity
    params.sim.ICs.dtheta_pend = 0; % initial pendulum rotational velocity
    params.sim.dt = 0.05;           % simulation timestep
end