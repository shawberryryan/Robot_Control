function xdot = filename(x0)

    syms q1 q2
    % Planar 2-link robot dimensions
    m1 = 7.848;
    m2 = 4.49;
    L1 = 0.3;
    Lc1 = 0.1554;
    Lc2 = 0.0341;
    I1 = 0.176;
    I2 = 0.0411;
    
    % Gain constants (Problem 1)
    kp1 = 100;
    kd1 = 20;
    kp2 = 100;
    kd2 = 20;
    
    % Control inputs
    T1 = kp1*(q1_d - q1) - kd1*q1dot;
    T2 = kp2*(q2_d - q2) - kd2*q2dot;

    % Intertia matrix terms
    d_11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2(t))) + I1 + I2;
    d_12 = m2*(Lc1^2 + L1*Lc2*cos(q2(t))) + I2;
    d_21 = d_12;
    d_22 = m2*Lc2^2+I2;
    
    delta = d_11*d_22 - d_21*d_12;
    
    % Double Derivative terms
    q_1_double_dot = (1/delta)*(d_22*a_1 - d_12*a_2);
    q_2_double_dot = (1/delta)*(-d_21*a_1 + d_11*a_2);
end