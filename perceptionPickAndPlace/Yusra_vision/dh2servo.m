function servoAngles = dh2servo(dhJointAngles)
    servoAngles = dhJointAngles;
    servoAngles(1) = dhJointAngles(1) - pi/2;
    servoAngles(2) = dhJointAngles(2) - pi/2;
end

%[appendix]{"version":"1.0"}
%---
