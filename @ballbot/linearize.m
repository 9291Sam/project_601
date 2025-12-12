function [A, B] = linearize(sys,x,u)
    zero_x = zeros(size(x));
    zero_u = zeros(size(u));
    if (isequal(zero_x(3:end), x(3:end)) && isequal(zero_u, u))
        A = get_A_matrix(sys.r, sys.d, sys.m_ball, sys.m_body, sys.I_ball, sys.I_body_xy, sys.I_body_z, sys.g);
        B = get_B_matrix(sys.r, sys.d, sys.m_ball, sys.m_body, sys.I_ball, sys.I_body_xy, sys.I_body_z, sys.g);
    else
        x
        u
        error("Can't linearize around given x and u!");
    end

end