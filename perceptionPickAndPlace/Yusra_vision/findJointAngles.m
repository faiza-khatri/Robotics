function solutions = findJointAngles(x, y, z, phi)
    % Robot link lengths (mm) based on your pincherFK parameters
    d1 = 14.121;
    a2 = 10.187;
    a3 = 10.187;
    a4 = 7.11;

    % Initialize solutions matrix (Max 4 solutions: 2 for theta1 * 2 for theta3)
    solutions = [];

    % 1. Solve for two possible theta1 solutions (Forward and Backward)
    theta1_options = [atan2(y, x), atan2(-y, -x)];

    for i = 1:2
        theta1 = theta1_options(i);
        
        % Horizontal distance from base to end-effector
        % If theta1 is "backward", r must be negative to reach the target point
        if i == 1
            r = sqrt(x^2 + y^2);
        else
            r = -sqrt(x^2 + y^2);
        end
        
        s = z - d1;

        % 2. Find coordinates of the wrist center (r_bar, s_bar)
        r_bar = r - a4 * cos(phi);
        s_bar = s - a4 * sin(phi);

        % 3. Solve for theta3 (Law of Cosines)
        % Using: r_bar^2 + s_bar^2 = a2^2 + a3^2 + 2*a2*a3*cos(theta3)
        cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);

        % Numerical robustness: clamp cos_theta3 to [-1, 1]
        if abs(cos_theta3) > 1 && abs(cos_theta3) < 1.01
            cos_theta3 = sign(cos_theta3);
        end
        
        if abs(cos_theta3) <= 1
            % Two possible elbow configurations: Elbow Up and Elbow Down
            theta3_options = [atan2(sqrt(1 - cos_theta3^2), cos_theta3), atan2(-sqrt(1 - cos_theta3^2), cos_theta3)];

            for j = 1:2
                theta3 = theta3_options(j);

                % 4. Solve for theta2
                theta2 = (atan2(s_bar, r_bar) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3)));

                % 5. Solve for theta4 based on orientation constraint
                % phi = theta2 + theta3 + theta4
                theta4 = phi - theta2 - theta3;

                % Append the valid solution to the matrix
                angles = dh2servo([theta1, theta2, theta3, theta4]);
                solutions = [solutions; angles];
            end
        end
    end
    
    if isempty(solutions)
        warning('Target position (%.2f, %.2f, %.2f) is out of reach.', x, y, z);
    end
end

%[appendix]{"version":"1.0"}
%---
