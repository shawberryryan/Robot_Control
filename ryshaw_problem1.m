% Control gains
kp1 = 100;
kd1 = 20;
kp2 = kp1;
kd2 = kd1;

% Initial conditions
x0 = [0.05; 0; 0.05; 0];  % Initial positions (q1, q2) and velocities (q1dot, q2dot)

% Time span of the simulation
tspan = [0 2];

% Solve the ODE
[T, X] = ode45(@PD_control, tspan, x0);

% Desired positions (changes at t = 1)
q1_d = (T < 1) * pi/2 + (T >= 1) * 0;
q2_d = (T < 1) * pi/2 + (T >= 1) * 0;


% Compute torques based on the controller design
T1 = max(-10, min(10, kp1*(q1_d - X(:,1)) - kd1*X(:,2)));
T2 = max(-10, min(10, kp2*(q2_d - X(:,3)) - kd2*X(:,4)));

% Plotting
% q_d
figure
plot(T, q1_d, 'b', T, q2_d, 'r');
title('Desired Angles');
legend('q1 desired', 'q2 desired');


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