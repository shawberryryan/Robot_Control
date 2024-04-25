function ryshaw_problem3
    % Initial conditions
    x0 = [0.05; 0; 0.05; 0];  % Initial positions (q1, q2) and velocities (q1dot, q2dot)

    % Time span of the simulation
    tspan = [0 2];

    % Solve the ODE
    [T, X] = ode45(@ID_control, tspan, x0);

    % Initialize for plotting
    q1_d = zeros(size(T));
    v1_d = zeros(size(T));
    a1_d = zeros(size(T));
    q2_d = zeros(size(T));
    v2_d = zeros(size(T));
    a2_d = zeros(size(T)); 
    

    % Recalculating torques 
    T1 = zeros(size(T));
    T2 = zeros(size(T));
    m1 = 7.848;
    m2 = 4.49;
    L1 = 0.3;
    Lc1 = 0.1554;
    Lc2 = 0.0341;
    I1 = 0.176;
    I2 = 0.0411;
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
        
        % IDC stuff again
        % Setup, gross
        d_11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2(i))) + I1 + I2;
        d_12 = m2*(Lc1^2 + L1*Lc2*cos(q2(i))) + I2;
        d_21 = d_12;
        d_22 = m2*Lc2^2+I2;
        
        % Matrices
        h = -m2*L1*Lc2*sin(q2(i)); % From SHV pg. 222
        C = [h*q2dot(i), (h*q2dot(i) + h*q1dot(i)); -h*q1dot(i), 0]; % Page 222 SHV
        D = [d_11, d_12; d_21, d_22];

        % Control law
        aq1 = a1_d(i) + kp1*(q1_d(i) - q1(i)) + kd1*(v1_d(i) - q1dot(i));
        aq2 = a2_d(i) + kp2*(q2_d(i) - q2(i)) + kd2*(v2_d(i) - q2dot(i));
        a_q = [aq1; aq2];
        dot_q = [q1dot(i); q2dot(i)];
        tau = D*a_q + C*dot_q;
        T1(i) = tau(1);
        T2(i) = tau(2);
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
    subplot(3,1,1); plot(T, q1_d); title('Desired Position q1_d'); xlabel('Time (s)'); ylabel('Position');
    subplot(3,1,2); plot(T, v1_d); title('Desired Velocity v1_d'); xlabel('Time (s)'); ylabel('Velocity');
    subplot(3,1,3); plot(T, a1_d); title('Desired Acceleration a1_d'); xlabel('Time (s)'); ylabel('Acceleration');

end




