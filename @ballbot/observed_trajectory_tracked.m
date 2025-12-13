function x = observed_trajectory_tracked(sys, t_span, x0, K, r, L, C, D)

    plant = @(x, u) sys.dynamics(x, u);

    observer = @(xhat, y, u) sys.dynamics(xhat, u) + L*(y-C*xhat-D*u);

    output = @(x, u) C*x+D*u; % adding in the D term makes this take forever?

    controller = @(t, x) -K*(x - r(t));

    function f = full_dynamics(t, combined_state)
        x = combined_state(1:8);
        xhat = combined_state(9:16);

        u = controller(t, xhat);

        u_limit = 100; 
        u = max(min(u, u_limit), -u_limit);

        y = output(x, u);

        f = [plant(x, u); observer(xhat, y, u)];
    end

    [~, x] = ode45(@(t, x) full_dynamics(t, x), t_span, x0);

end