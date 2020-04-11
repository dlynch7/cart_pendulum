%% control_PMP.m
%
% Description:
%   Wrapper function for autogen_control_PMP.m
%   Computes control u from robot state x and costate lambda.
%
% Inputs:
%
% Outputs:
%   u: (scalar) control effort (horizontal force applied to cart)

function u = control_PMP(x,lambda,params)

    % break up state "x" into generalized coordinates and velocities:
    x_cart      = x(1);
    theta_pend  = x(2);
    dx_cart     = x(3);
    dtheta_pend = x(4);

    u = autogen_control_PMP(params.model.dyn.pend.I,...
                            params.control.swingup.TPBVP.R,...
                            lambda(3),...
                            lambda(4),...
                            params.model.dyn.cart.m,...
                            params.model.dyn.pend.m,...
                            params.model.dyn.pend.r_com,...
                            theta_pend);

end