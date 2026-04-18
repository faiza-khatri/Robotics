% function solutions = findJointAngles(x,y,z,phi)
%     d1 = 0.045; 
%     a2 = 0.1035; a3 = 0.1035; a4 = 0.11;
% 
%     theta1_solutions = [atan2(y, x), atan2(y, x)+pi];
% 
% 
% 
%     solutions = zeros(4, 4);
%     idx = 1;
%     for i = 1:length(theta1_solutions)
%         theta1 = theta1_solutions(i);
% 
%         % projection into the rotated frame
%         r = x*cos(theta1) + y*sin(theta1);
%         s = z - d1;
% 
%         % wrist center decoupling
%         r_bar = r - a4 * cos(phi);
%         s_bar = s - a4 * sin(phi);
% 
%         cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);
%         cos_theta3 = max(-1, min(1, cos_theta3));  % clamp to [-1, 1]
%         if abs(cos_theta3) > 1
%             solutions = [];
%             disp('No solution exists: target out of reach');
%             return;
%         end
%         theta3_solutions = [acos(cos_theta3), -acos(cos_theta3)];
%         for j = 1:length(theta3_solutions)
%             theta3 = theta3_solutions(j);
%             gamma = atan2(s_bar, r_bar);
%             theta2_geom = gamma - atan2(a3*sin(theta3), a2 + a3*cos(theta3));
%             % theta2_geom = atan2(s_bar, r_bar) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
%             if r_bar < 0
%                 theta2_geom = theta2_geom + pi;
%             end
%             theta2 = theta2_geom - pi/2;  
%             theta4 = phi - theta2_geom - theta3;
%             solutions(idx, :) = [theta1, theta2, theta3, theta4];
%             idx = idx + 1;
%         end
%     end
%     solutions = atan2(sin(solutions), cos(solutions));
% end
% function solutions = findJointAngles(x, y, z, phi)
%     d1 = 0.045;
%     a2 = 0.1035; a3 = 0.1035; a4 = 0.11;
% 
%     t1_base = atan2(y, x);  % in (-pi, pi]
% 
%     if abs(x) < 1e-9 && abs(y) < 1e-9
%         theta1_candidates = [0, pi];
%     else
%         t1_other = t1_base + pi;
%         if t1_other > pi
%             t1_other = t1_other - 2*pi;  % wrap back into (-pi, pi]
%         end
%         theta1_candidates = [t1_base, t1_other];
%     end
% 
%     solutions = zeros(4, 4);
%     idx = 1;
% 
%     for i = 1:length(theta1_candidates)
%         theta1 = theta1_candidates(i);
% 
%         r = x*cos(theta1) + y*sin(theta1);
%         s = z - d1;
% 
%         r_bar = r - a4 * cos(phi);
%         s_bar = s - a4 * sin(phi);
% 
%         cos_theta3 = (r_bar^2 + s_bar^2 - a2^2 - a3^2) / (2 * a2 * a3);
% 
%         if abs(cos_theta3) > 1 + 1e-9
%             continue;
%         end
%         cos_theta3 = max(-1, min(1, cos_theta3));
% 
%         theta3_candidates = [acos(cos_theta3), -acos(cos_theta3)];
% 
%         for j = 1:length(theta3_candidates)
%             theta3 = theta3_candidates(j);
% 
%             theta2_geom = atan2(s_bar, r_bar) - atan2(a3*sin(theta3), a2 + a3*cos(theta3));
%             theta2 = theta2_geom - pi/2;
%             theta4 = phi - theta2_geom - theta3;
% 
%             solutions(idx, :) = [theta1, theta2, theta3, theta4];
%             idx = idx + 1;
%         end
%     end
% 
%     solutions = solutions(1:idx-1, :);
%     solutions = atan2(sin(solutions), cos(solutions));
% end

function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
%task 5.6
function [x,y,z,R] = pincherFK(jointAngles)

    %jointAngles is a 1x4 vector of joint angles in radians in DH convention
    %outputs of this function are x,y,z -> end-effector position 
    % and R -> end-effector orientation

    L1 = 0.045;%meters
    L2 = 0.1035;   
    L3 = 0.1035;  
    L4 = 0.11;   

    % theta of each joint is given as argument
    theta1 = jointAngles(1);
    theta2 = jointAngles(2);
    theta3 = jointAngles(3);
    theta4 = jointAngles(4);
    
    %calculating intermediate transformations
    A01 = dh_transform(0, pi/2, L1, theta1);
    A12 = dh_transform(L2, 0, 0, theta2 + pi/2);
    A23 = dh_transform(L3, 0, 0, theta3);
    A34 = dh_transform(L4, 0, 0, theta4);

    
    %resultant transformation
    T04 = A01 * A12 * A23 * A34;

    p = T04(1:3,4); % end effector position
    R = T04(1:3,1:3); % end effector orientation

    x = p(1); y = p(2); z = p(3);

end
% Generate valid test points from known joint configurations
test_configs = [
    0, pi/2, 0, pi/2;
    pi/2, pi/2, 0, pi/2;
];

for i = 1:size(test_configs, 1)
    q = test_configs(i,:);
    [x,y,z,~] = pincherFK(q);
    % compute phi from joint angles
    phi = (q(2) + pi/2) + q(3) + q(4);
    fprintf('Config %d: q=[%.4f %.4f %.4f %.4f] -> x=%.4f y=%.4f z=%.4f phi=%.4f\n', ...
             i, q(1), q(2), q(3), q(4), x, y, z, phi);
    solutions = findJointAngles(x, y, z, phi);
    for j=1:size(solutions, 1)
        [xf, yf, zf, ~] = pincherFK(solutions(j, :));
        fprintf('Solution %d: theta=[%.4f %.4f %.4f %.4f] -> xf=%.4f yf=%.4f zf=%.4f\n', ...
                 j, solutions(j, 1), solutions(j, 2), solutions(j, 3), solutions(j, 4), xf, yf, zf);
    end
    fprintf('\n');

end
