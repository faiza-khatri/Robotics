%task 5.5
%arb = Arbotix('port', 'COM5', 'nservos', 5)
syms theta1 theta2 theta3 theta4 real

L1 = 0.045;%meters
L2 = 0.1035;   
L3 = 0.1035;  
L4 = 0.11;   

function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

T01 = dh_transform(0, -pi/2, L1, theta1);
T12 = dh_transform(L2, 0, 0, theta2);
T23 = dh_transform(L3, 0, 0, theta3);
T34 = dh_transform(L4, 0, 0, theta4);

T04 = simplify(T01 * T12 * T23 * T34);

p = T04(1:3,4); %end effector position
disp('Position of end-effector:')
disp(p)

R = T04(1:3,1:3); %end effector orientation
disp('Rotation matrix:')
disp(R)
