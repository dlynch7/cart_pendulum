function fixed_time_tpbvp_ode = autogen_fixed_time_tpbvp_ode(I_pend,Q_swingup1_1,Q_swingup1_2,Q_swingup1_3,Q_swingup1_4,Q_swingup2_1,Q_swingup2_2,Q_swingup2_3,Q_swingup2_4,Q_swingup3_1,Q_swingup3_2,Q_swingup3_3,Q_swingup3_4,Q_swingup4_1,Q_swingup4_2,Q_swingup4_3,Q_swingup4_4,R_swingup1,b1,b2,dtheta_pend,dx_cart,g,lambda1,lambda2,lambda3,lambda4,m_cart,m_pend,r_com_pend,theta_pend,x_cart)
%AUTOGEN_FIXED_TIME_TPBVP_ODE
%    FIXED_TIME_TPBVP_ODE = AUTOGEN_FIXED_TIME_TPBVP_ODE(I_PEND,Q_SWINGUP1_1,Q_SWINGUP1_2,Q_SWINGUP1_3,Q_SWINGUP1_4,Q_SWINGUP2_1,Q_SWINGUP2_2,Q_SWINGUP2_3,Q_SWINGUP2_4,Q_SWINGUP3_1,Q_SWINGUP3_2,Q_SWINGUP3_3,Q_SWINGUP3_4,Q_SWINGUP4_1,Q_SWINGUP4_2,Q_SWINGUP4_3,Q_SWINGUP4_4,R_SWINGUP1,B1,B2,DTHETA_PEND,DX_CART,G,LAMBDA1,LAMBDA2,LAMBDA3,LAMBDA4,M_CART,M_PEND,R_COM_PEND,THETA_PEND,X_CART)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    10-Apr-2020 11:52:54

t2 = r_com_pend.^2;
t3 = cos(theta_pend);
t4 = m_pend.^2;
t5 = I_pend.*m_cart;
t6 = I_pend.*m_pend;
t7 = m_cart.*m_pend.*t2;
t8 = sin(theta_pend);
t9 = dtheta_pend.^2;
t10 = 1.0./R_swingup1;
t11 = I_pend.*lambda3;
t12 = lambda3.*m_pend.*t2;
t13 = lambda4.*m_pend.*r_com_pend.*t8;
t14 = t11+t12+t13;
t15 = t2.*t4;
t16 = t8.^2;
t23 = t2.*t4.*t16;
t17 = t5+t6+t7+t15-t23;
t18 = 1.0./t17;
t19 = t3.^2;
t20 = t2.*t4.*t19;
t21 = t5+t6+t7+t20;
t22 = 1.0./t21;
t24 = I_pend.*b1.*dx_cart;
t25 = b1.*dx_cart.*m_pend.*t2;
t26 = g.*t2.*t3.*t4.*t8;
t27 = b2.*dtheta_pend.*m_pend.*r_com_pend.*t8;
t28 = t19.*2.0;
t29 = t28-1.0;
t30 = 1.0./t21.^2;
t31 = b2.*dtheta_pend.*m_cart;
t32 = b2.*dtheta_pend.*m_pend;
t33 = g.*r_com_pend.*t3.*t4;
t34 = g.*m_cart.*m_pend.*r_com_pend.*t3;
t35 = b1.*dx_cart.*m_pend.*r_com_pend.*t8;
t36 = 1.0./t17.^3;
t37 = m_pend.*t2;
t38 = I_pend+t37;
fixed_time_tpbvp_ode = [dx_cart;dtheta_pend;-t22.*(t24+t25+t26+t27+I_pend.*t10.*t14.*t18-I_pend.*m_pend.*r_com_pend.*t3.*t9.*2.0+m_pend.*t2.*t10.*t14.*t18-r_com_pend.*t2.*t3.*t4.*t9.*2.0);-t22.*(t31+t32+t33+t34+t35-t2.*t3.*t4.*t8.*t9.*2.0+m_pend.*r_com_pend.*t8.*t10.*t14.*t18);Q_swingup1_4.*dtheta_pend.*(-1.0./2.0)-(Q_swingup4_1.*dtheta_pend)./2.0-(Q_swingup1_3.*dx_cart)./2.0-(Q_swingup3_1.*dx_cart)./2.0-(Q_swingup1_2.*theta_pend)./2.0-(Q_swingup2_1.*theta_pend)./2.0-Q_swingup1_1.*x_cart;Q_swingup2_4.*dtheta_pend.*(-1.0./2.0)-(Q_swingup4_2.*dtheta_pend)./2.0-(Q_swingup2_3.*dx_cart)./2.0-(Q_swingup3_2.*dx_cart)./2.0-Q_swingup2_2.*theta_pend+lambda4.*(-m_pend.*r_com_pend.*t22.*(-b1.*dx_cart.*t3+g.*m_cart.*t8+g.*m_pend.*t8+m_pend.*r_com_pend.*t9.*t29.*2.0)+t2.*t3.*t4.*t8.*t30.*(t31+t32+t33+t34+t35-t2.*t3.*t4.*t8.*t9.*2.0).*2.0+m_pend.*r_com_pend.*t3.*t10.*t14.*t18.*t22+m_pend.*r_com_pend.*t2.*t3.*t4.*t10.*t14.*t16.*t36.*2.0)-(Q_swingup1_2.*x_cart)./2.0-(Q_swingup2_1.*x_cart)./2.0+lambda3.*(m_pend.*r_com_pend.*t22.*(I_pend.*t8.*t9.*2.0+b2.*dtheta_pend.*t3+g.*m_pend.*r_com_pend.*t29+m_pend.*t2.*t8.*t9.*2.0)+t2.*t3.*t4.*t8.*t30.*(t24+t25+t26+t27-I_pend.*m_pend.*r_com_pend.*t3.*t9.*2.0-r_com_pend.*t2.*t3.*t4.*t9.*2.0).*2.0+t2.*t3.*t4.*t8.*t10.*t14.*t36.*t38.*2.0);-lambda1-(Q_swingup3_4.*dtheta_pend)./2.0-(Q_swingup4_3.*dtheta_pend)./2.0-Q_swingup3_3.*dx_cart-(Q_swingup2_3.*theta_pend)./2.0-(Q_swingup3_2.*theta_pend)./2.0-(Q_swingup1_3.*x_cart)./2.0-(Q_swingup3_1.*x_cart)./2.0+b1.*lambda3.*t22.*t38+b1.*lambda4.*m_pend.*r_com_pend.*t8.*t22;-lambda2-Q_swingup4_4.*dtheta_pend-(Q_swingup3_4.*dx_cart)./2.0-(Q_swingup4_3.*dx_cart)./2.0-(Q_swingup2_4.*theta_pend)./2.0-(Q_swingup4_2.*theta_pend)./2.0-(Q_swingup1_4.*x_cart)./2.0-(Q_swingup4_1.*x_cart)./2.0+lambda4.*t22.*(b2.*m_cart+b2.*m_pend-dtheta_pend.*t2.*t4.*sin(theta_pend.*2.0).*2.0)-lambda3.*m_pend.*r_com_pend.*t22.*(-b2.*t8+I_pend.*dtheta_pend.*t3.*4.0+dtheta_pend.*m_pend.*t2.*t3.*4.0)];
