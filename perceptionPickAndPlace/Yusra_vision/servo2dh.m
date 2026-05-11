function dhJointAngles = servo2dh(jointAngles)
    dhJointAngles = jointAngles;
    dhJointAngles(1) = jointAngles(1) +pi/2;
    dhJointAngles(2) = jointAngles(2) +pi/2;
end

%[appendix]{"version":"1.0"}
%---
