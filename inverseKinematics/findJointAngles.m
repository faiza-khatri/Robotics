function solutions = findJointAngles(x,y,z,phi)
    d1 = 0.045; 
    a2 = 0.1035; a3 = 0.1035; a4 = 0.11;

    r = sqrt(x^2 + y^2);
    s = z - d1;
    
    r_bar = r - a4 * cos(phi);
    s_bar = s - a4 * sin(phi);

    cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);
    cos_theta3 = max(-1, min(1, cos_theta3));  % clamp to [-1, 1]

    % theta3 solutions
    
    if abs(cos_theta3) > 1
        solutions = [];
        disp('No solution exists: target out of reach');
        return;
    end
    theta3_solutions = [acos(cos_theta3), -acos(cos_theta3)];
    theta1_solutions = [atan2(y, x), -2*pi + atan2(y, x)];

    solutions = zeros(4, 4);
    idx = 1;
    for i = 1:length(theta1_solutions)
        theta1 = theta1_solutions(i);
        for j = 1:length(theta3_solutions)
            theta3 = theta3_solutions(j);
            theta2_geom = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
            theta2 = theta2_geom - pi/2;  
            theta4 = phi - theta2_geom - theta3;
            solutions(idx, :) = [theta1, theta2, theta3, theta4];
            idx = idx + 1;
        end
    end
    solutions = atan2(sin(solutions), cos(solutions));
end