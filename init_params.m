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
    params.model.dyn.cart.m = 0.5;    % mass of the cart
    params.model.dyn.pend.m = 0.2;    % mass of the pendulum
    params.model.dyn.pend.I = 0.006;% moment of inertia of the pendulum
    params.model.dyn.pend.r_com = 0.3; % radial loc. of pendulum CoM
    params.model.dyn.g = 9.81;      % acceleration due to gravity
    params.model.dyn.b1 = 0.1;      % damping between cart and track
    params.model.dyn.b2 = 0.0;     % damping between pendulum and cart
    
    % parameters that help with visualizing the robot:
    params.model.geom.cart.w = 0.25; % width of the cart
    params.model.geom.cart.h = 0.25; % height of the cart
    params.model.geom.pend.l = 0.5;   % length of the pendulum
    params.model.geom.pend.w = 0.05; % width of the pendulum
    
    params.viz.colors.cart = [0.5 0.5 0.5];
    params.viz.colors.pend = [0.25 0.25 0.25];
    params.viz.colors.pend_com = [0.75 0.75 0.75];
    params.viz.colors.tracers.cart_com = 'r';
    params.viz.colors.tracers.pend_com = 'g';
    params.viz.colors.tracers.pend_tip = 'b';
    params.viz.axis_lims = [-1,1,-0.5,0.5];
    
    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.x_cart = 0.2;      % initial cart position
    params.sim.ICs.theta_pend = 0.8*pi/2;  % initial pendulum angle
    params.sim.ICs.dx_cart = 0;     % initial cart velocity
    params.sim.ICs.dtheta_pend = 0; % initial pendulum rotational velocity
    params.sim.dt = 0.05;           % simulation timestep
    
    % parameters related to control
    params.control.inverted.K = zeros(4,4); % state feedback gain matrix
    params.control.swingup.K = zeros(4,4); % state feedback gain matrix
end