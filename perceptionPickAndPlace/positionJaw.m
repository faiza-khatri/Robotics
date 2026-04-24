function success = positionJaw(arb, position)
    
    min_pos = 7;
    max_pos = 20;

    %check validity
    if position > max_pos || position < min_pos
        success = false; % if position is out of limits, set success to false
        return;
    end

     %  calibration data 
    angleDeg = [150 147.07 120.70 106.05 91.40 79.68 62.10 47.46 32.81 ...
            4.68 -25.7 -53.90 -87.89 -110.74 -150];

    jawPos = [6.62 6.865 7.155 8.115 8.5 9.15 9.995 11.02 11.97 ...
          14.04 16.3 17.95 20.045 21 21.575];

    % interpolate position → angle
    thetaDeg = interp1(jawPos, angleDeg, position, 'pchip');

    if isnan(thetaDeg)
        success = false;
        return;
    end

    % convert to radians 
    thetaRad = deg2rad(thetaDeg);

    % send command to gripper motor 
    try
        arb.setpos(5, thetaRad, 12);
        success = true;
    catch
        success = false;
    end  

    success = true;  % if position is within limits, set success to true
end

arb = Arbotix('port', 'COM12', 'nservos', 5);
success = positionJaw(arb, 7.5)
