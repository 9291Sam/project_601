function [x] = simulate_no_input(sys, t, x0)

[~, x] = ode45(@(t, x) sys.dynamics_no_input(x), t, x0);


end