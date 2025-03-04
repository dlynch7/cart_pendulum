function H = autogen_control_hamiltonian(I_pend,Q_swingup1_1,Q_swingup1_2,Q_swingup1_3,Q_swingup1_4,Q_swingup2_1,Q_swingup2_2,Q_swingup2_3,Q_swingup2_4,Q_swingup3_1,Q_swingup3_2,Q_swingup3_3,Q_swingup3_4,Q_swingup4_1,Q_swingup4_2,Q_swingup4_3,Q_swingup4_4,R_swingup1,b1,b2,dtheta_pend,dx_cart,g,lambda1,lambda2,lambda3,lambda4,m_cart,m_pend,r_com_pend,theta_pend,u,x_cart)
%AUTOGEN_CONTROL_HAMILTONIAN
%    H = AUTOGEN_CONTROL_HAMILTONIAN(I_PEND,Q_SWINGUP1_1,Q_SWINGUP1_2,Q_SWINGUP1_3,Q_SWINGUP1_4,Q_SWINGUP2_1,Q_SWINGUP2_2,Q_SWINGUP2_3,Q_SWINGUP2_4,Q_SWINGUP3_1,Q_SWINGUP3_2,Q_SWINGUP3_3,Q_SWINGUP3_4,Q_SWINGUP4_1,Q_SWINGUP4_2,Q_SWINGUP4_3,Q_SWINGUP4_4,R_SWINGUP1,B1,B2,DTHETA_PEND,DX_CART,G,LAMBDA1,LAMBDA2,LAMBDA3,LAMBDA4,M_CART,M_PEND,R_COM_PEND,THETA_PEND,U,X_CART)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    10-Apr-2020 19:21:05

t2 = r_com_pend.^2;
t3 = cos(theta_pend);
t4 = m_pend.^2;
t5 = dtheta_pend.^2;
t6 = sin(theta_pend);
t7 = I_pend.*m_cart;
t8 = I_pend.*m_pend;
t9 = m_cart.*m_pend.*t2;
t10 = t3.^2;
t11 = t2.*t4.*t10;
t12 = t7+t8+t9+t11;
t13 = 1.0./t12;
H = dtheta_pend.*lambda2+dx_cart.*lambda1+(R_swingup1.*u.^2)./2.0+dtheta_pend.*((Q_swingup4_4.*dtheta_pend)./2.0+(Q_swingup3_4.*dx_cart)./2.0+(Q_swingup2_4.*theta_pend)./2.0+(Q_swingup1_4.*x_cart)./2.0)+dx_cart.*((Q_swingup4_3.*dtheta_pend)./2.0+(Q_swingup3_3.*dx_cart)./2.0+(Q_swingup2_3.*theta_pend)./2.0+(Q_swingup1_3.*x_cart)./2.0)+theta_pend.*((Q_swingup4_2.*dtheta_pend)./2.0+(Q_swingup3_2.*dx_cart)./2.0+(Q_swingup2_2.*theta_pend)./2.0+(Q_swingup1_2.*x_cart)./2.0)+x_cart.*((Q_swingup4_1.*dtheta_pend)./2.0+(Q_swingup3_1.*dx_cart)./2.0+(Q_swingup2_1.*theta_pend)./2.0+(Q_swingup1_1.*x_cart)./2.0)+lambda3.*t13.*(I_pend.*u-I_pend.*b1.*dx_cart+m_pend.*t2.*u-b1.*dx_cart.*m_pend.*t2+I_pend.*m_pend.*r_com_pend.*t3.*t5.*2.0-b2.*dtheta_pend.*m_pend.*r_com_pend.*t6-g.*t2.*t3.*t4.*t6+r_com_pend.*t2.*t3.*t4.*t5.*2.0)-lambda4.*t13.*(b2.*dtheta_pend.*m_cart+b2.*dtheta_pend.*m_pend+g.*r_com_pend.*t3.*t4-m_pend.*r_com_pend.*t6.*u+b1.*dx_cart.*m_pend.*r_com_pend.*t6+g.*m_cart.*m_pend.*r_com_pend.*t3-t2.*t3.*t4.*t5.*t6.*2.0);
