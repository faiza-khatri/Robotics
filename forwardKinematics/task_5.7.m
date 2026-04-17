%task 5.7
function dhJointAngles = servo2dh(jointAngles)
    
    % offset: dh_angle = servo_angle + offset
    offset = [0,0, 0, 0];   % in radians (since we have already accomodated dh offset in its theta definition, we can have offset 0 here and get correct ans)
    sign  = [1, 1, 1, 1];   % depends on rotation direction
    dhJointAngles = zeros(1,4); %initializing a zero vector 

    %calculating dh joint angles
    dhJointAngles(1) = sign(1) .* jointAngles(1) + offset(1);
    dhJointAngles(2) = sign(2) .* jointAngles(2) + offset(2);
    dhJointAngles(3) = sign(3) .* jointAngles(3) + offset(3);
    dhJointAngles(4) = sign(4) .* jointAngles(4) + offset(4);

end

dhJointAngles = servo2dh([0 0 0 0])
