function currentConfig = getCurrentPose(arb)
    try
        currentConfig = arb.getpos(); 
    catch
        warning('Failed to read from Arbotix. Returning home position.');
        currentConfig = [0, 0, 0, 0];
    end
end

%[appendix]{"version":"1.0"}
%---
