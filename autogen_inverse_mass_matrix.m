function Minv = autogen_inverse_mass_matrix(I_pend,m_cart,m_pend,r_com_pend,theta_pend)
%AUTOGEN_INVERSE_MASS_MATRIX
%    MINV = AUTOGEN_INVERSE_MASS_MATRIX(I_PEND,M_CART,M_PEND,R_COM_PEND,THETA_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    09-Apr-2020 17:31:19

t2 = r_com_pend.^2;
t3 = m_pend.^2;
t4 = sin(theta_pend);
t5 = I_pend.*m_cart;
t6 = I_pend.*m_pend;
t7 = t2.*t3;
t8 = m_cart.*m_pend.*t2;
t9 = t4.^2;
t12 = t2.*t3.*t9;
t10 = t5+t6+t7+t8-t12;
t11 = 1.0./t10;
t13 = m_pend.*r_com_pend.*t4.*t11;
Minv = reshape([t11.*(I_pend+m_pend.*t2),t13,t13,t11.*(m_cart+m_pend)],[2,2]);
