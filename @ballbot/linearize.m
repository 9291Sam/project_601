function [A, B] = linearize(sys,x,u)

    A = get_A_matrix(sys.r, sys.d, sys.m_ball, sys.m_body, sys.I_ball, sys.I_body_xy, sys.I_body_z, sys.g);
    B = get_B_matrix(sys.r, sys.d, sys.m_ball, sys.m_body, sys.I_ball, sys.I_body_xy, sys.I_body_z, sys.g);

end