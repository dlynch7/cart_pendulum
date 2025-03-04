%% grad_tpbvp_ode.m
%
% Description:
%   Wrapper function for autogen_grad_tpbvp_ode.m
%   Computes the gradient* of the vector field for the two-point boundary 
%   value problem (TPBVP) that results from applying Pontryagin's Maximum 
%   Principle (PMP) to the cart-pendulum swingup task with quadratic 
%   running cost
%       x'*Q*x + u'*R*u
%
%   *the gradient with respect to z, which is an 8x1 column vector
%   (fixed-time TPBVP) or a 9x1 column vector (free-time TPBVP). z is
%   defined below in the "Inputs" section of the Description.
%
% Inputs:
%   t: (scalar) time (required by MATLAB's numerical integrators/solvers)
%   z: (8x1 or 9x1 column vector). If z is 8x1 (fixed-time TPBVP), then:
%       z = [x;lambda] where the robot state is
%       x = [q; dq]
%         = [x_cart; theta_pend; dx_cart; dtheta_pend] and where the
%       costate is
%           lambda = [lambda1; lambda2; lambda3; lambda4].
%       If z is 9x1 (free-time TPBVP), then:
%       z = [x;lambda;rho] where the robot state is
%       x = [q; dq]
%         = [x_cart; theta_pend; dx_cart; dtheta_pend]; where the
%       costate is
%           lambda = [lambda1; lambda2; lambda3; lambda4]; and where "rho"
%           is a dummy scalar variable that is required to solve a
%           free-time TPBVP with MATLAB's bvp4c or bvp5c solver.
%   params: a struct with many elements, generated by calling init_params.m
%
% Outputs:
%   dz: the right-hand side of the system of 1st-order ODEs that govern the
%       TPBVP dynamics. dz has same shape as z (8x1)

function [grad_dz] = grad_tpbvp_ode(t,z,params)

% break up "z" into robot state "x" and costate "lambda":
x = z(1:4);
lambda = z(5:8);

% break up state "x" into generalized coordinates and velocities:
x_cart      = x(1);
theta_pend  = x(2);
dx_cart     = x(3);
dtheta_pend = x(4);

   
if numel(z)==8 % fixed-time TPBVP
    grad_dz = autogen_grad_tpbvp_fixed_time_ode(params.model.dyn.pend.I,...
                                 params.control.swingup.TPBVP.Q(1,1),...
                                 params.control.swingup.TPBVP.Q(1,2),...
                                 params.control.swingup.TPBVP.Q(1,3),...
                                 params.control.swingup.TPBVP.Q(1,4),...
                                 params.control.swingup.TPBVP.Q(2,1),...
                                 params.control.swingup.TPBVP.Q(2,2),...
                                 params.control.swingup.TPBVP.Q(2,3),...
                                 params.control.swingup.TPBVP.Q(2,4),...
                                 params.control.swingup.TPBVP.Q(3,1),...
                                 params.control.swingup.TPBVP.Q(3,2),...
                                 params.control.swingup.TPBVP.Q(3,3),...
                                 params.control.swingup.TPBVP.Q(3,4),...
                                 params.control.swingup.TPBVP.Q(4,1),...
                                 params.control.swingup.TPBVP.Q(4,2),...
                                 params.control.swingup.TPBVP.Q(4,3),...
                                 params.control.swingup.TPBVP.Q(4,4),...
                                 params.control.swingup.TPBVP.R,...
                                 params.model.dyn.b1,...
                                 params.model.dyn.b2,...
                                 dtheta_pend,...
                                 dx_cart,...
                                 params.model.dyn.g,...
                                 lambda(3),...
                                 lambda(4),...
                                 params.model.dyn.cart.m,...
                                 params.model.dyn.pend.m,...
                                 params.model.dyn.pend.r_com,...
                                 theta_pend);
elseif numel(z)==9 % free-time TPBVP
    rho = z(9); % time scaling variable
    grad_dz = autogen_grad_tpbvp_free_time_ode(params.model.dyn.pend.I,...
                                 params.control.swingup.TPBVP.Q(1,1),...
                                 params.control.swingup.TPBVP.Q(1,2),...
                                 params.control.swingup.TPBVP.Q(1,3),...
                                 params.control.swingup.TPBVP.Q(1,4),...
                                 params.control.swingup.TPBVP.Q(2,1),...
                                 params.control.swingup.TPBVP.Q(2,2),...
                                 params.control.swingup.TPBVP.Q(2,3),...
                                 params.control.swingup.TPBVP.Q(2,4),...
                                 params.control.swingup.TPBVP.Q(3,1),...
                                 params.control.swingup.TPBVP.Q(3,2),...
                                 params.control.swingup.TPBVP.Q(3,3),...
                                 params.control.swingup.TPBVP.Q(3,4),...
                                 params.control.swingup.TPBVP.Q(4,1),...
                                 params.control.swingup.TPBVP.Q(4,2),...
                                 params.control.swingup.TPBVP.Q(4,3),...
                                 params.control.swingup.TPBVP.Q(4,4),...
                                 params.control.swingup.TPBVP.R,...
                                 params.model.dyn.b1,...
                                 params.model.dyn.b2,...
                                 dtheta_pend,...
                                 dx_cart,...
                                 params.model.dyn.g,...
                                 lambda(1),...
                                 lambda(2),...
                                 lambda(3),...
                                 lambda(4),...
                                 params.model.dyn.cart.m,...
                                 params.model.dyn.pend.m,...
                                 params.model.dyn.pend.r_com,...
                                 rho,...
                                 theta_pend,...
                                 x_cart);
    
else % z has invalid dimensions
    errormsg = sprintf('TPBVP state vector z has dimensions %d-by-%d.\n',...
        size(z));
    error(errormsg);
end

end