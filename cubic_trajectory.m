function [q1_d, v1_d, a1_d, q2_d, v2_d, a2_d] = cubic_trajectory(t)
    % Define acceleration profiles for a1_d and a2_d
    a = 6*pi;
    y0 = 3*pi;
    
    % Piecwise offsets
    x = 2*a-y0; % accel offset at t = 1
    y = y0 + x - a; % velocity offset
    z = y + (a/3)-(x/2 + y0/2); % pos offset

    if t < 1
        a1_d = y0 - a*t;
        v1_d = y0*t - (a/2)*t.^2;
        q1_d = (y0/2)*t.^2 - (a/6)*t^3; 
    else
        a1_d = a*t - x;
        v1_d = (a/2)*t.^2 - x*t + y;
        q1_d = (a/6)*t.^3 - (x/2)*t.^2 + y*t - z;
    end

    a2_d = a1_d;
    v2_d = v1_d;
    q2_d = q1_d;
end
