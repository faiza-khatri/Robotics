function pointcloud_example()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    
    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();


    %% Acquire device parameters 
    % Get streaming device's name
    dev = profile.get_device();  

    % Access Depth Sensor
    depth_sensor = dev.first('depth_sensor');

    % Find the mapping from 1 depth unit to meters, i.e. 1 depth unit =
    % depth_scaling meters.
    depth_scaling = depth_sensor.get_depth_scale();

    % Extract the depth stream
    depth_stream = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    
    % Get the intrinsics
    depth_intrinsics = depth_stream.get_intrinsics();
    % to also get color intrinsics:
    color_stream = profile.get_stream(realsense.stream.color).as('video_stream_profile');
    color_intrinsics = color_stream.get_intrinsics();

    %% Align the frames and then get the frames
    % Get frames. We discard the first couple to allow
    % the camera time to settle
    for i = 1:8
        fs = pipe.wait_for_frames();
    end
    
    % Alignment is necessary as the depth cameras and RGB cameras are
    % physically separated. So, the same (x,y,z) in real world maps to
    % different (u,v) in the depth image and the color images. To build a
    % point cloud we only need depth image, but if we want the color the
    % cloud then we'll need the other image.

    % Since the two images are of different sizes, we can either align the
    % depth to color image, or the color to depth.
    % Change the argument to realsense.stream.color to align to the color
    % image.
    % align_to_depth = realsense.align(realsense.stream.depth);
    align_to_color = realsense.align(realsense.stream.color); % I want it to align to color as my coordiantes [u v] will be from color image
    
    fs = align_to_color.process(fs);
    
    % Stop streaming
    pipe.stop();

    % Extract the depth frame
    depth = fs.get_depth_frame();
    depth_data = double(depth.get_data());
    depth_frame = permute(reshape(depth_data',[ depth.get_width(),depth.get_height()]),[2 1]);

    % Extract the color frame
    color = fs.get_color_frame();    
    color_data = color.get_data();
    color_frame = permute(reshape(color_data',[3,color.get_width(),color.get_height()]),[3 2 1]);

    %% Create a point cloud using MATLAB library
    % Create a MATLAB intrinsics object
    % intrinsics = cameraIntrinsics([depth_intrinsics.fx,depth_intrinsics.fy],[depth_intrinsics.ppx,depth_intrinsics.ppy],size(depth_frame));
    % ptCloud = pcfromdepth(depth_frame,1/depth_scaling,intrinsics,ColorImage=color_frame);

    intrinsics = cameraIntrinsics([color_intrinsics.fx, color_intrinsics.fy], ...
                                  [color_intrinsics.ppx, color_intrinsics.ppy], ...
                                  [double(color_intrinsics.height), double(color_intrinsics.width)]);
    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ColorImage=color_frame);
    
    figure; pcshow(ptCloud);
    title('Point Cloud');
    
    maxDistance = 0.008;   % tune this
    [planeModel, inlierIdx, outlierIdx] = pcfitplane(ptCloud, maxDistance, ...
        'MaxNumTrials', 2000, ...       % more RANSAC iterations = better plane fit
        'Confidence', 99.5);

    % Remove table
    objectsCloud = select(ptCloud, outlierIdx);
    figure; pcshow(objectsCloud);
    title('Plane removed');
    % 1. Segment the ground points

    % [groundPtsIdx, nonGroundPtCloud, groundPtCloud] = segmentGroundSMRF(ptCloud);
    
    % % 2. Visualize the result
    % pcshowpair(nonGroundPtCloud, groundPtCloud)
    % title('Non-Ground (Green) and Ground (Red)')

    

    colors = objectsCloud.Color;
    hsvVals = rgb2hsv(im2double(colors));
    
    H = hsvVals(:,1);
    S = hsvVals(:,2);
    V = hsvVals(:,3);
    
    redMask = (H < 0.05 | H > 0.95) & S > 0.5 & V > 0.2;
    
    redCloud = select(objectsCloud, find(redMask));
    redCloud = pcdenoise(redCloud);
    figure; pcshow(redCloud);
    title('Red blocks extracted');

    minDistance = 0.02;   % tunable
    [labels, numClusters] = pcsegdist(redCloud, minDistance);

    clusterSizes = zeros(numClusters,1);

    for i = 1:numClusters
        clusterSizes(i) = sum(labels == i);
    end
    
    % Sort clusters by size
    [sortedSizes, sortedIdx] = sort(clusterSizes,'descend');


    minPoints = 500;   % tune this

    numBlocksExpected = 3;

    figure;
    
    for k = 1:numBlocksExpected
        
        i = sortedIdx(k);
        idx = find(labels == i);
        blockCloud = select(redCloud, idx);
        
        pts = blockCloud.Location;
        pts = reshape(pts, [], 3);
        
        % Remove NaNs
        pts = pts(~any(isnan(pts),2),:);

        % ---- Position ----
        position = mean(pts,1);
        % position(3) = 0.4597;
        
        % ---- Orientation via PCA ----
        ptsCentered = pts - position;

        [coeff, ~, ~] = pca(ptsCentered);

        % --- FIX: Constrain Z axis to table normal ---
        % PCA axis 3 (least variance) should be the surface normal,
        % but its sign and exact direction are arbitrary. Replace it
        % with the known table normal instead.

        tableNormal = planeModel.Normal;           % unit normal from RANSAC fit
        tableNormal = tableNormal / norm(tableNormal);

        % The table normal points toward the camera (positive Z in camera frame).
        % Ensure it points AWAY from the table (upward toward camera).
        % The block centroid is above the table, so the vector from any table
        % inlier point to the centroid should agree with the normal direction.
        % Any point on the table plane: use the plane equation directly.
        % For plane ax+by+cz+d=0, a point on it is -d*n (along the normal).
        params   = planeModel.Parameters;             % [a b c d]
        tablePt  = -params(4) * params(1:3);          % 1x3 point guaranteed on plane

        toBlock  = position - tablePt;
        if dot(tableNormal, toBlock) > 0
            tableNormal = -tableNormal;
        end

        % Use PCA axes 1 and 2 as the X/Y of the block face,
        % but re-orthogonalize them against the true Z (table normal).
        % coeff(:,1) = dominant in-plane axis (long axis of block)
        % coeff(:,2) = secondary in-plane axis
        Zaxis = tableNormal';                          % 3x1, forced to table normal

        % Re-orthogonalize X against Z via Gram-Schmidt
        Xaxis = coeff(:,1) - dot(coeff(:,1), Zaxis) * Zaxis;
        Xaxis = Xaxis / norm(Xaxis);

        % Y = Z cross X to keep right-handed frame
        Yaxis = cross(Zaxis, Xaxis);
        Yaxis = Yaxis / norm(Yaxis);

        R = [Xaxis, Yaxis, Zaxis];   % columns are the frame axes

        
        % R = coeff;   % 3x3 rotation matrix

        % Ensure right-handed coordinate frame
        % if det(R) < 0
        %     R(:,3) = -R(:,3);
        % end
        eul = rotm2eul(R);   % [yaw pitch roll] in radians
        eul_deg = rad2deg(eul);

        T = eye(4);
        T(1:3,1:3) = R;
        T(1:3,4) = position';
        
              
        pcshow(redCloud);
        hold on;
        
        % Plot centroid
        plot3(position(1), position(2), position(3), ...
          'o', ...
          'MarkerSize',10, ...
          'MarkerEdgeColor','y', ...
          'MarkerFaceColor','y');

        
        % Plot orientation axes
        scale = 0.2;
        
        quiver3(position(1),position(2),position(3), ...
                scale*R(1,1),scale*R(2,1),scale*R(3,1), ...
                'r','LineWidth',2);
        
        quiver3(position(1),position(2),position(3), ...
                scale*R(1,2),scale*R(2,2),scale*R(3,2), ...
                'g','LineWidth',2);
        
        quiver3(position(1),position(2),position(3), ...
                scale*R(1,3),scale*R(2,3),scale*R(3,3), ...
                'b','LineWidth',2);
        
        title('Pose Verification (PCA)');

        fprintf('\n============================\n');
        fprintf('Block %d\n', k);
        fprintf('============================\n');
        
        fprintf('Position (meters):\n');
        fprintf('x = %.4f\n', position(1));
        fprintf('y = %.4f\n', position(2));
        fprintf('z = %.4f\n\n', position(3));
        
        fprintf('Rotation Matrix (R):\n');
        disp(R);
        
        fprintf('Euler Angles ZYX (degrees):\n');
        fprintf('Yaw   = %.2f\n', eul_deg(1));
        fprintf('Pitch = %.2f\n', eul_deg(2));
        fprintf('Roll  = %.2f\n\n', eul_deg(3));
        
        fprintf('Homogeneous Transform (T):\n');
        disp(T);


    end

        hold off;





    % Display point cloud
    % figure; pcshow(ptCloud,'VerticalAxisDir','Down');
    %figure; pcshow(inlierPoints,'VerticalAxisDir','Down');
    %figure; pcshow(outlierPoints,'VerticalAxisDir','Down');

    
    
end
