function [x] = simulate(sys,t,x0)

[~,x] = ode45(@(t,x) sys.dynamics(x),t,x0);

end