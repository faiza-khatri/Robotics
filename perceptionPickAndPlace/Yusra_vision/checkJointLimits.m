function isValid = checkJointLimits(servo_angles)
    limit = deg2rad(150);
    isValid = all(servo_angles >= -limit & servo_angles <= limit);
end

%[appendix]{"version":"1.0"}
%---
