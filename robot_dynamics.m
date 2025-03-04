%% robot_dynamics.m
%
% Description:
%   Computes the right-hand side of the state-space dynamics
%       dx = f_ss(x) + g_ss(x)*u
%
% Inputs:
%   t: time (scalar)
%   x: the state vector, x = [x_cart; theta_pend; dx_cart; dtheta_pend];
%   u_ff: feedforward control input (scalar, horizontal force applied to
%       the cart)
%   params: a struct with many elements, generated by calling init_params.m
%   varargin: optional name-value pair arguments:
%       'controller': (default: 'passive') can have three values:
%           'passive': no feedback control, but calling function can still
%               supply a feedforward control u_ff;
%           'stabilize': a feedback controller is used to stabilize the
%               inverted equilibrium,
%               gain matrix stored in params.control.inverted.K;
%           'swingup': an energy-shaping feedback controller is used to
%               bring the system to a homoclinic orbit,
%               gain matrix stored in params.control.swingup.K;
%
% Outputs:
%   dx: derivative of state x with respect to time.

function [dx] = robot_dynamics(t,x,u_ff,params,varargin)

% Parse input arguments
% Note: a simple robot dynamics function doesn't need this, but I want to
% write extensible code, so I'm using "varargin" which requires input
% parsing. See the reference below:
%
% https://people.umass.edu/whopper/posts/better-matlab-functions-with-the-inputparser-class/

% Step 1: instantiate an inputParser:
p = inputParser;

% Step 2: create the parsing schema:
%   2a: required inputs:
addRequired(p,'cart_pend_time',...
    @(t) isnumeric(t) && size(t,1)==1);
addRequired(p,'cart_pend_state', ...
    @(x) isnumeric(x) && size(x,1)==4 && size(x,2)==1);
addRequired(p,'cart_pend_u_ff',...
    @(u_ff) isnumeric(u_ff) && size(u_ff,1)==1 && size(u_ff,2)==1);
addRequired(p,'cart_pend_params', ...
    @(params) ~isempty(params));
%   2b: optional inputs:
%       optional name-value pairs to trace different parts of the robot:
addParameter(p, 'controller', 'passive');

% Step 3: parse the inputs:
parse(p, t,x,u_ff,params,varargin{:});
% disp(p.Results)

% Finally, actually compute the controls + dynamics:
    switch p.Results.controller
        case 'passive'
            u_fb = 0;
        case 'stabilize'
            x_ref = params.control.inverted.x_eq;
            u_fb = -params.control.inverted.K*(x - x_ref);
        case 'swingup'
            u_fb = 0;
        otherwise
            u_fb = 0;
    end

    u = u_ff + u_fb;

    f_ss = drift_vector_field(x,params);
    g_ss = control_vector_field(x,params);
    dx = f_ss + g_ss*u;

end