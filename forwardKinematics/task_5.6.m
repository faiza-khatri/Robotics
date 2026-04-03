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
    T01 = dh_transform(0, -pi/2, L1, theta1);
    T12 = dh_transform(L2, 0, 0, theta2);
    T23 = dh_transform(L3, 0, 0, theta3);
    T34 = dh_transform(L4, 0, 0, theta4);
    
    %resultant transformation
    T04 = simplify(T01 * T12 * T23 * T34);

    p = T04(1:3,4); % end effector position
    R = T04(1:3,1:3); % end effector orientation

end
