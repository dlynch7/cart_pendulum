function f_ss = autogen_drift_vector_field(I_pend,b1,b2,dtheta_pend,dx_cart,g,m_cart,m_pend,r_com_pend,theta_pend)
%AUTOGEN_DRIFT_VECTOR_FIELD
%    F_SS = AUTOGEN_DRIFT_VECTOR_FIELD(I_PEND,B1,B2,DTHETA_PEND,DX_CART,G,M_CART,M_PEND,R_COM_PEND,THETA_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    10-Apr-2020 11:52:38

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
f_ss = [dx_cart;dtheta_pend;-t13.*(I_pend.*b1.*dx_cart+b1.*dx_cart.*m_pend.*t2-I_pend.*m_pend.*r_com_pend.*t3.*t5.*2.0+b2.*dtheta_pend.*m_pend.*r_com_pend.*t6+g.*t2.*t3.*t4.*t6-r_com_pend.*t2.*t3.*t4.*t5.*2.0);-t13.*(b2.*dtheta_pend.*m_cart+b2.*dtheta_pend.*m_pend+g.*r_com_pend.*t3.*t4+b1.*dx_cart.*m_pend.*r_com_pend.*t6+g.*m_cart.*m_pend.*r_com_pend.*t3-t2.*t3.*t4.*t5.*t6.*2.0)];
