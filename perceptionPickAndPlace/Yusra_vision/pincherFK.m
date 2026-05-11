function [x,y,z,R,phi] = pincherFK(jointAngles)
   jointAngles = servo2dh(jointAngles);
   % DH Parameters [a, alpha, d, theta]
   DH = [0,       pi/2,  14.121, jointAngles(1);
        10.187,   0,     0,       jointAngles(2);
        10.187,   0,     0,       jointAngles(3);
        6.62,     0,     0,       jointAngles(4)];
    
    % syms('theta_1');
    % syms('theta_2');
    % syms('theta_3');syms('theta_4');
    % DH Parameters [a, alpha, d, theta]
    % DH = [0,       -pi/2,  141.21,  theta_1;
    %       101.87,   0,     0,       theta_2;
    %       101.87,   0,     0,       theta_3;
    %       66.2,     0,     0,       theta_4];

   % Initialize Identity Matrix for 0T4
    T04 = eye(4);
    T_cell = cell(1, 4);
    
    for i = 1:4
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        th = DH(i,4);
        
        T_cell{i} = [cos(th), -sin(th)*cos(alpha),  sin(th)*sin(alpha), a*cos(th);
                     sin(th),  cos(th)*cos(alpha), -cos(th)*sin(alpha), a*sin(th);
                     0,        sin(alpha),          cos(alpha),         d;
                     0,        0,                   0,                  1];
      
        T04 = T04 * T_cell{i};
    end

    x = T04(1,4)
    y = T04(2,4)
    z = T04(3,4)
    R = T04(1:3, 1:3);
    phi = DH(2,4) + DH(3,4) + DH(4,4)
end

%[appendix]{"version":"1.0"}
%---
