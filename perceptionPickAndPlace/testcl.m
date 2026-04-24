function diagnoseTransform()
    % Run cameraToObject() first, then call this
    cTo = cameraToObject();
    
    %% Try different T_wc and see which gives sensible world coords
    
    % Your physical measurements:
    tx = 0.0;   % MEASURE THIS
    ty = 0.0;   % MEASURE THIS  
    tz = 0.46;   % MEASURE THIS (camera height above table + table height above base)
    
    % For downward-facing camera: try all 4 USB-orientation variants
    R_options = {
        [ 1  0  0;  0 -1  0;  0  0 -1],  'USB toward robot';
        [-1  0  0;  0  1  0;  0  0 -1],  'USB away from robot';
        [ 0 -1  0; -1  0  0;  0  0 -1],  'USB to left';
        [ 0  1  0;  1  0  0;  0  0 -1],  'USB to right'
    };
    
    for n = 1:4
        R_wc = R_options{n,1};
        label = R_options{n,2};
        
        T_wc = eye(4);
        T_wc(1:3,1:3) = R_wc;
        T_wc(1:3,4)   = [tx; ty; tz];
        
        T_co = squeeze(cTo(1,:,:));
        T_wo = T_wc * T_co;
        
        p = T_wo(1:3,4);
        reach = sqrt(p(1)^2 + p(2)^2);
        
        fprintf('\n--- %s ---\n', label);
        fprintf('World pos: x=%.3f  y=%.3f  z=%.3f\n', p(1), p(2), p(3));
        fprintf('Horizontal reach needed: %.4f m  (max: 0.207 m)\n', reach);
        fprintf('z above base: %.4f m  (should be small, ~0.0-0.05 m)\n', p(3));
        
        if reach < 0.20 && p(3) > -0.01 && p(3) < 0.10
            fprintf('*** THIS ORIENTATION LOOKS CORRECT ***\n');
        end
    end
end

diagnoseTransform()