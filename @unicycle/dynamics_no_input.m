function dx = dynamics_no_input(sys, x)




% when simulating, n
% EoM = M(q,dq)*qdd + f(q,dq) == 0;
%  M(q,dq)*qdd == -f(q,dq);
% qdd = M(q, dq)^-1 * -f(q, dq)


M = getMassMatrix(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);
f = getFVector(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);
% B = getInputMatrix(x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12), sys.r, sys.d, sys.m_w, sys.m_s, sys.I_wpitch, sys.I_wyawroll, sys.g);

dx(1:6) = x(7:12);
dx(7:12) = M\-f;

dx = dx';

end