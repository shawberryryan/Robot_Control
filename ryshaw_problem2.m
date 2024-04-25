function ryshaw_problem2
    % Initial conditions
    x0 = [0.05; 0; 0.05; 0];  % Initial positions (q1, q2) and velocities (q1dot, q2dot)

    % Time span of the simulation
    tspan = [0 2];

    % Solve the ODE
    [T, X] = ode45(@PD_feedforward, tspan, x0);

    % Initialize for plotting
    q1_d = zeros(size(T));
    v1_d = zeros(size(T));
    a1_d = zeros(size(T));
    q2_d = zeros(size(T));
    v2_d = zeros(size(T));
    a2_d = zeros(size(T)); 
    
    T1 = zeros(size(T));
    T2 = zeros(size(T));
    kp1 = 100;
    kp2 = kp1;
    kd1 = 20;
    kd2 = kd1;
    q1 = X(:,1);
    q1dot = X(:,2);
    q2 = X(:,3);
    q2dot = X(:,4);

    for i = 1:length(T)
        [q1_d(i), v1_d(i), a1_d(i), q2_d(i), v2_d(i), a2_d(i)] = cubic_trajectory(T(i));
        T1(i) = max(-10, min(10, a1_d(i) + kp1*(q1_d(i)-q1(i))+kd1*(v1_d(i)-q1dot(i))));
        T2(i) = max(-10, min(10, a2_d(i) + kp2*(q2_d(i)-q2(i))+kd2*(v2_d(i)-q2dot(i))));
    end
    
    % Plotting
    figure;
    plot(T, X(:,1), 'b-', T, X(:,3), 'r--');
    ylabel('Joint Angles (rad)');
    title('Joint Responses');
    legend('q1', 'q2');
    
    figure;
    plot(T, X(:,1)-q1_d, 'b-', T, X(:,3)-q2_d, 'r--');
    ylabel('Error (rad)');
    title('Tracking Errors');
    legend('Error q1', 'Error q2');
    
    figure;
    plot(T, T1, 'b-', T, T2, 'r--');
    ylabel('Torque (Nm)');
    xlabel('Time (s)');
    title('Joint Torques');
    legend('Torque q1', 'Torque q2');

    figure;
    plot(T, q1_d); title('Desired Position q1_d'); xlabel('Time (s)'); ylabel('Position');
    figure; 
    plot(T, v1_d); title('Desired Velocity v1_d'); xlabel('Time (s)'); ylabel('Velocity');
    figure;
    plot(T, a1_d); title('Desired Acceleration a1_d'); xlabel('Time (s)'); ylabel('Acceleration');

end




