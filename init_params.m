%% init_params 
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
    params.model.dyn.m_cart = 1;    % mass of the cart
    params.model.dyn.m_pend = 1;    % mass of the pendulum
    params.model.dyn.I_pend = 0.001;% moment of inertia of the pendulum
    params.model.dyn.r_com_pend = 0.5; % radial loc. of pendulum CoM
    params.model.dyn.g = 9.81;      % acceleration due to gravity
    params.model.dyn.b1 = 0.0;      % damping between cart and track
    params.model.dyn.b2 = 0.05;     % damping between pendulum and cart
    
    % parameters that help with visualizing the robot:
    params.model.geom.l_pend = 1;   % length of the pendulum
    params.model.geom.w_pend = 0.1; % width of the pendulum
    params.model.geom.w_cart = 0.5; % width of the cart
    params.model.geom.h_cart = 0.5; % height of the cart
    
    % parameters related to simulating (integrating) the dynamics forward
    % in time:
    params.sim.ICs.x_cart = 0;      % initial cart position
    params.sim.ICs.theta_pend = 0;  % initial pendulum angle
    params.sim.ICs.dx_cart = 0;     % initial cart velocity
    params.sim.ICs.dtheta_pend = 0; % initial pendulum rotational velocity
    params.sim.dt = 0.05;           % simulation timestep
end