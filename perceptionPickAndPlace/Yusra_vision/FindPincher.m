function [x,y,z,R, phi] = FindPincher(arb)
    [x,y,z,R, phi] = pincherFK(servo2dh(arb.getpos()));
end

%[appendix]{"version":"1.0"}
%---
