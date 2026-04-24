function errorCode = setPosition(jointAngles)
% Accepts joint angles in radians, sets them as goal positions for motors.
% errorCode = 1 : success, all joints set successfully
% errorCode = 0 : failure, a joint angle was out of range [-140, 140] degrees
% we are setting lim [-140, 140] as to be more cautious
    minAngle = -2.44; maxAngle = 2.44;
    for i = 1:length(jointAngles)
        if jointAngles(i) > maxAngle || jointAngles(i) < minAngle  % check limits in radians
            errorCode = 0;  % failure
            fprintf('Error: Joint %d angle (%.2f°) is out of range [%d°, %d°]\n', ...
                    i, jointAngles(i), minAngle, maxAngle);
            return;
        else
            arb.setpos(i, jointAngles(i), 20);
            errorCode = 1;  % success
        end
    end
end
