function M = autogen_mass_matrix(I_pend,m_cart,m_pend,r_com_pend,theta_pend)
%AUTOGEN_MASS_MATRIX
%    M = AUTOGEN_MASS_MATRIX(I_PEND,M_CART,M_PEND,R_COM_PEND,THETA_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    09-Apr-2020 17:31:19

t2 = sin(theta_pend);
M = reshape([m_cart+m_pend,-m_pend.*r_com_pend.*t2,-m_pend.*r_com_pend.*t2,I_pend+m_pend.*r_com_pend.^2],[2,2]);
