% since we have FOUR solutions, the output is a 4x4 matrix
function solutions = findJointAngles(x,y,z,phi)
    d1 = 0.045; 
    a2 = 0.1035; a3 = 0.1035; a4 = 0.11;

    r = sqrt(x^2 + y^2);
    s = z - d1;
    
    r_bar = r - a4 * cos(phi);
    s_bar = s - a4 * sin(phi);

    cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

    % theta3 solutions
    
    if abs(cos_theta3) > 1
        solutions = [];
        disp('No solution exists: target out of reach');
        return;
    end
    theta3_solutions = [acos(cos_theta3), -acos(cos_theta3)];
    theta1_solutions = [atan2(y, x), atan2(-y, -x)];

    solutions = zeros(4, 4);
    idx = 1;
    for i = 1:length(theta1_solutions)
        theta1 = theta1_solutions(i);
        for j = 1:length(theta3_solutions)
            theta3 = theta3_solutions(j);
            theta2 = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
            theta4 = phi - theta2 - theta3;
            solutions(idx, :) = [theta1, theta2, theta3, theta4];
            idx = idx + 1;
        end
    end
    solutions = arctan(sin(solution), cos(solutions));
end
solutions = zeros(4, 4);
x = 0.02; y = 0.02; z = 0.02;
phi = 0;
solutions = findJointAngles(x, y, z, phi)
