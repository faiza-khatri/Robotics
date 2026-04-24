function solutions = findJointAngles(x, y, z, phi)
    d1 = 0.045;
    a2 = 0.1035; a3 = 0.1035; a4 = 0.11;

    t1_base = atan2(y, x);  % in (-pi, pi]

    if abs(x) < 1e-9 && abs(y) < 1e-9
        theta1_candidates = [0, pi];
    else
        t1_other = t1_base + pi;
        if t1_other > pi
            t1_other = t1_other - 2*pi;  % wrap back into (-pi, pi]
        end
        theta1_candidates = [t1_base, t1_other];
    end

    solutions = zeros(4, 4);
    idx = 1;

    for i = 1:length(theta1_candidates)
        theta1 = theta1_candidates(i);

        r = x*cos(theta1) + y*sin(theta1);
        s = z - d1;

        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);
        reach = sqrt(r_bar^2 + s_bar^2);
        maxReach = a2 + a3;
        
        fprintf('Required reach: %.4f m\n', reach);
        fprintf('Max reach:      %.4f m\n', maxReach);
        fprintf('cos_theta3:     %.4f\n', (r_bar^2 + s_bar^2 - a2^2 - a3^2)/(2*a2*a3));

        cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        if abs(cos_theta3) > 1 + 1e-9
            continue;
        end
        cos_theta3 = max(-1, min(1, cos_theta3));

        theta3_candidates = [acos(cos_theta3), -acos(cos_theta3)];

        for j = 1:length(theta3_candidates)
            theta3 = theta3_candidates(j);

            theta2_geom = atan2(s_bar, r_bar) - atan2(a3*sin(theta3), a2 + a3*cos(theta3));
            theta2 = theta2_geom - pi/2;
            theta4 = phi - theta2_geom - theta3;

            solutions(idx, :) = [theta1, theta2, theta3, theta4];
            idx = idx + 1;
        end
    end

    solutions = solutions(1:idx-1, :);
    solutions = atan2(sin(solutions), cos(solutions));
end