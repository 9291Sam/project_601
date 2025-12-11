function [A, B] = linearize(sys, x, u)
    % Linearizes the unicycle dynamics around state x and input u
    % using the symbolically generated Jacobian functions.
    
    % Call the generated functions.
    % Arguments must match the 'Vars' list in eom_deriver:
    % {z_sym, u_sym, r, d, m_w, m_s, I_wpitch, I_wyawroll, g}
    
    A = getA_Advanced(x, u, sys.r, sys.d, sys.m_w, sys.m_s, ...
                      sys.I_wpitch, sys.I_wyawroll, sys.g);
                      
    B = getB_Advanced(x, u, sys.r, sys.d, sys.m_w, sys.m_s, ...
                      sys.I_wpitch, sys.I_wyawroll, sys.g);
end