function xdot = PD_control(t, x)
    % Planar 2-link robot dimensions
    m1 = 7.848;
    m2 = 4.49;
    L1 = 0.3;
    Lc1 = 0.1554;
    Lc2 = 0.0341;
    I1 = 0.176;
    I2 = 0.0411;

    % Control gains
    kp1 = 100;
    kd1 = 20;
    kp2 = kp1;
    kd2 = kd1;

    % Extracting state variables
    q1 = x(1);  % Current position for joint 1
    q1dot = x(2); % Current velocity for joint 1
    q2 = x(3);  % Current position for joint 2
    q2dot = x(4); % Current velocity for joint 2

    % Desired positions (setpoints)
    if t < 1
        q1_d = pi/2; % Initial desired position for q1
        q2_d = pi/2; % Initial desired position for q2
    else
        q1_d = 0; % Change to new desired position for q1 after t=1
        q2_d = 0; % Change to new desired position for q2 after t=1
    end

    % Intertia matrix terms
    d_11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2)) + I1 + I2;
    d_12 = m2*(Lc1^2 + L1*Lc2*cos(q2)) + I2;
    d_21 = d_12;
    d_22 = m2*Lc2^2+I2;
    delta = d_11*d_22 - d_21*d_12; % Determinant of D(q)

    % Coriolis terms
    h = -m2*L1*Lc2*sin(q2); % From SHV pg. 222
    c121 = h;
    c211 = 0;
    c221 = h;   
    c112 = -h;
    
    % Control inputs, range limited to -10 <= Ti <= 10
    T1 = max(-10, min(10, kp1*(q1_d - q1) - kd1*q1dot));
    T2 = max(-10, min(10, kp2*(q2_d - q2) - kd2*q2dot));


    a1 = T1 - c121*q1dot*q2dot - c211*q2dot*q1dot - c221*q2dot^2;
    a2 = T2 - c112*q1dot^2;
    
    % Double Derivative terms
    q1doubledot = (1/delta)*(d_22*a1 - d_12*a2);
    q2doubledot = (1/delta)*(-d_21*a1 + d_11*a2);

    xdot = [q1dot; q1doubledot; q2dot; q2doubledot];
end