function ryshaw_problem2
    % Robot parameters
    params = struct('m1', 7.848, 'm2', 4.49, ...
                    'L1', 0.3, 'Lc1', 0.1554, 'Lc2', 0.0341, ...
                    'I1', 0.176, 'I2', 0.0411, ...
                    'kp1', 100, 'kd1', 20, ...
                    'kp2', 100, 'kd2', 20);

    % Initial conditions
    x0 = [0; 0; 0; 0];  % Initial positions (q1, q2) and velocities (q1dot, q2dot)

    % Time span of the simulation
    tspan = [0 2];

    % Solve the ODE
    [T, X] = ode45(@(t, x) ode_func(t, x, params), tspan, x0);

    % Initialize for plotting
    q1_d = zeros(length(T));
    v1_d = zeros(length(T));
    a1_d = zeros(length(T));
    q2_d = zeros(length(T));
    v2_d = zeros(length(T));
    a2_d = zeros(length(T));
    T1 = zeros(length(T));
    T2 = zeros(length(T));
    kp1 = 100;
    kp2 = kp1;
    kd1 = 20;
    kd2 = kd1;
    q1 = X(:,1);
    q1dot = X(:,2);
    q2 = X(:,3);
    q2dot = X(:,4);

    % Compute the desired trajectory at each time step
    for i = 1:length(T)
        [q1_d(i), v1_d(i), a1_d(i), q2_d(i), v2_d(i), a2_d(i)] = cubic_trajectory(T(i));
        %T1(i) = max(-10, min(10, a1_d(i) + kp1*(q1_d(i)-q1(I))+kd1*(v1_d(i)-q1dot(i))));
        %T2(i) = max(-10, min(10, a2_d(i) + kp2*(q2_d(i)-q2)+kd2*(v2_d(i)-q2dot(i))));
    end
    T1 = max(-10, min(10, a1_d + kp1*(q1_d-q1)+kd1*(v1_d-q1dot)));
    T2 = max(-10, min(10, a2_d + kp2*(q2_d-q2)+kd2*(v2_d-q2dot)));
    
    % Plotting
    figure;
    subplot(3,1,1);
    plot(T, X(:,1), 'b-', T, X(:,3), 'r--');
    ylabel('Joint Angles (rad)');
    title('Joint Responses');
    legend('q1', 'q2');
    
    subplot(3,1,2);
    plot(T, X(:,1)-q1_d, 'b-', T, X(:,3)-q2_d, 'r--');
    ylabel('Error (rad)');
    title('Tracking Errors');
    legend('Error q1', 'Error q2');
    
    subplot(3,1,3);
    plot(T, T1, 'b-', T, T2, 'r--');
    ylabel('Torque (Nm)');
    xlabel('Time (s)');
    title('Joint Torques');
    legend('Torque q1', 'Torque q2');

    figure;
    subplot(3,1,1); plot(T, q1_d); title('Desired Position q1_d'); xlabel('Time (s)'); ylabel('Position');
    subplot(3,1,2); plot(T, v1_d); title('Desired Velocity v1_d'); xlabel('Time (s)'); ylabel('Velocity');
    subplot(3,1,3); plot(T, a1_d); title('Desired Acceleration a1_d'); xlabel('Time (s)'); ylabel('Acceleration');


end

function xdot = ode_func(t, x, params)
    % Compute the desired trajectory at time t
    [q1_d, v1_d, a1_d, q2_d, v2_d, a2_d] = cubic_trajectory(t);

    % Call robot_dynamics with the desired values
    xdot = PD_feedforward(t, x, params, q1_d, v1_d, a1_d, q2_d, v2_d, a2_d);
end


