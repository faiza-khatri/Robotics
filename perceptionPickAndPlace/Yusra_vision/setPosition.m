function errorCode = setPosition(arb, jointAngles)
    isValid = checkJointLimits(jointAngles);
    if ~isValid
        warning("Joint angles out of range\n");
        errorCode = 1;
        return;
    end
    
    jointAngles = jointAngles(:)'; 
    n = numel(jointAngles);
    
    if n < 4 || n > 4
        warning('setPosition: Expected %d joint angles, got %d.', 4, n);
        errorCode = 2;
        return;
    end
    
    
    try
       for k = 1:4
          arb.setpos(k, jointAngles_rad(k), 50);
       end
        errorCode = 0;
        fprintf('setPosition: All joints commanded successfully.\n');
    catch e
        warning('setPosition: Motor command failed%d — %s', k, e.message);
        errorCode = 3;
    end
end