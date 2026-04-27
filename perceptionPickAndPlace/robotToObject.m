function wTo = robotToObject(cTo)
    %% T_world_camera: camera pose in robot base frame
    % You get this from hand-eye calibration (do this once, hardcode result).
    % Example values — REPLACE with your actual calibration result.
    
    % Rotation: camera optical axis pointing down at table
    % Translation: camera mounted ~0.8m above table origin
    R_wc = [ 0  -1  0;   % camera y is along our base x, camera x is along our base y, camera z is along our base -ve z
             -1 0  0;
             0  0 -1];
    
    distanceFromCamera = 0.46;
    x_offset = 0.112;
    y_offset = 0.003;
    t_wc = [x_offset; y_offset; distanceFromCamera]; % meters
    
    T_wc = eye(4);
    T_wc(1:3,1:3) = R_wc;
    T_wc(1:3,4)   = t_wc;
    
    %% Chain transforms for each block
    numBlocks = size(cTo, 1);
    wTo = zeros(numBlocks, 4, 4);
    
    for k = 1:numBlocks
        T_co = squeeze(cTo(k,:,:));   % 4x4 camera-to-object for block k
        T_wo = T_wc * T_co;       % world-to-object
        wTo(k,:,:) = T_wo;
        
        fprintf('Block %d in World Frame\n', k);

        pos_world = T_wo(1:3, 4);
        R_world   = T_wo(1:3,1:3);
        eul       = rotm2eul(R_world);
        eul_deg   = rad2deg(eul);
        
        fprintf('Position (meters):\n');
        fprintf('  x = %.4f\n', pos_world(1));
        fprintf('  y = %.4f\n', pos_world(2));
        fprintf('  z = %.4f\n\n', pos_world(3));
        fprintf('Euler Angles ZYX (degrees):\n');
        fprintf('  Yaw   = %.2f\n', eul_deg(1));
        fprintf('  Pitch = %.2f\n', eul_deg(2));
        fprintf('  Roll  = %.2f\n\n', eul_deg(3));
        fprintf('Full Transform T_world_object:\n');
        disp(T_wo);
    end
end