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

