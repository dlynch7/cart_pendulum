function B = autogen_upright_input_matrix(I_pend,m_cart,m_pend,r_com_pend)
%AUTOGEN_UPRIGHT_INPUT_MATRIX
%    B = AUTOGEN_UPRIGHT_INPUT_MATRIX(I_PEND,M_CART,M_PEND,R_COM_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    10-Apr-2020 11:52:39

t2 = r_com_pend.^2;
t3 = I_pend.*m_cart;
t4 = I_pend.*m_pend;
t5 = m_cart.*m_pend.*t2;
t6 = t3+t4+t5;
t7 = 1.0./t6;
B = [0.0;0.0;t7.*(I_pend+m_pend.*t2);m_pend.*r_com_pend.*t7];
