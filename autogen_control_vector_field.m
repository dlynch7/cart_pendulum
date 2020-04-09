function g_ss = autogen_control_vector_field(I_pend,m_cart,m_pend,r_com_pend,theta_pend)
%AUTOGEN_CONTROL_VECTOR_FIELD
%    G_SS = AUTOGEN_CONTROL_VECTOR_FIELD(I_PEND,M_CART,M_PEND,R_COM_PEND,THETA_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    09-Apr-2020 17:31:20

t2 = r_com_pend.^2;
t3 = m_pend.^2;
t4 = sin(theta_pend);
t5 = I_pend.*m_cart;
t6 = I_pend.*m_pend;
t7 = t2.*t3;
t8 = m_cart.*m_pend.*t2;
t9 = t4.^2;
t10 = t5+t6+t7+t8-t2.*t3.*t9;
t11 = 1.0./t10;
g_ss = [0.0;0.0;t11.*(I_pend+m_pend.*t2);m_pend.*r_com_pend.*t4.*t11];
