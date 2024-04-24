function xdot = PD_feedforward(t, x, params, q1_d, v1_d, a1_d, q2_d, v2_d, a2_d)
    % Planar 2-link robot dimensions
    m1 = params.m1;
    m2 = params.m2;
    L1 = params.L1;
    Lc1 = params.Lc1;
    Lc2 = params.Lc2;
    I1 = params.I1;
    I2 = params.I2;

    % Control gains
    kp1 = params.kp1;
    kd1 = params.kd1;
    kp2 = params.kp2;
    kd2 = params.kd2;

    % Extracting state variables
    q1 = x(1);  % Current position for joint 1
    q1dot = x(2); % Current velocity for joint 1
    q2 = x(3);  % Current position for joint 2
    q2dot = x(4); % Current velocity for joint 2

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
    T1 = max(-10, min(10, a1_d + kp1*(q1_d-q1)+kd1*(v1_d-q1dot)));
    T2 = max(-10, min(10, a2_d + kp2*(q2_d-q2)+kd2*(v2_d-q2dot)));


    a1 = T1 - c121*q1dot*q2dot - c211*q2dot*q1dot - c221*q2dot^2;
    a2 = T2 - c112*q1dot^2;
    
    % Double Derivative terms
    q1doubledot = (1/delta)*(d_22*a1 - d_12*a2);
    q2doubledot = (1/delta)*(-d_21*a1 + d_11*a2);

    xdot = [q1dot; q1doubledot; q2dot; q2doubledot];
end