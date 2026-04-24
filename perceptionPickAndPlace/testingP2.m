function pointcloud_merged()

    %% Setup
    pipe       = realsense.pipeline();
    colorizer  = realsense.colorizer();
    pcl_obj    = realsense.pointcloud();

    profile    = pipe.start();

    align_to_color = realsense.align(realsense.stream.color);

    dev        = profile.get_device();
    name       = dev.get_info(realsense.camera_info.name);
    fprintf('Device: %s\n', name);

    depth_sensor     = dev.first('depth_sensor');
    depth_scaling    = depth_sensor.get_depth_scale();

    depth_stream     = profile.get_stream(realsense.stream.depth).as('video_stream_profile');
    depth_intrinsics = depth_stream.get_intrinsics();

    %% Settle camera (discard first 8 frames)
    for i = 1:8
        fs = pipe.wait_for_frames();
    end

    %% Capture one aligned frameset (Code 1 accuracy approach)
    fs             = pipe.wait_for_frames();
    aligned_frames = align_to_color.process(fs);

    depth = aligned_frames.get_depth_frame();   % aligned depth
    color = fs.get_color_frame();               % original color frame

    pipe.stop();

    %% Extract depth frame
    depth_data  = double(depth.get_data());
    depth_frame = permute(reshape(depth_data', [depth.get_width(), depth.get_height()]), [2 1]);

    %% Extract color frame
    color_data  = color.get_data();
    color_frame = permute(reshape(color_data', [3, color.get_width(), color.get_height()]), [3 2 1]);

    %% Build point cloud via MATLAB intrinsics
    intrinsics = cameraIntrinsics( ...
        [depth_intrinsics.fx, depth_intrinsics.fy], ...
        [depth_intrinsics.ppx, depth_intrinsics.ppy], ...
        size(depth_frame));

    ptCloud = pcfromdepth(depth_frame, 1/depth_scaling, intrinsics, ColorImage=color_frame);

    figure; pcshow(ptCloud);
    title('Full Point Cloud');

    %% Remove dominant plane (table)
    maxDistance = 0.015;
    [planeModel, ~, outlierIdx] = pcfitplane(ptCloud, maxDistance);
    objectsCloud = select(ptCloud, outlierIdx);

    figure; pcshow(objectsCloud);
    title('Plane Removed');

    %% Extract red objects via HSV masking
    colors  = objectsCloud.Color;
    hsvVals = rgb2hsv(im2double(colors));
    H = hsvVals(:,1);
    S = hsvVals(:,2);
    V = hsvVals(:,3);

    redMask  = (H < 0.05 | H > 0.95) & S > 0.5 & V > 0.2;
    redCloud = select(objectsCloud, find(redMask));
    redCloud = pcdenoise(redCloud);

    figure; pcshow(redCloud);
    title('Red Blocks Extracted');

    %% Cluster red points
    minDistance = 0.02;
    [labels, numClusters] = pcsegdist(redCloud, minDistance);

    clusterSizes = zeros(numClusters, 1);
    for i = 1:numClusters
        clusterSizes(i) = sum(labels == i);
    end
    [~, sortedIdx] = sort(clusterSizes, 'descend');

    numBlocksExpected = 2;

    %% Define global reference axes for consistent PCA sign convention
    % Camera looks along +Z. Table normal from plane fit (points toward camera).
    % We want:
    %   block Z-axis  -> aligned with table normal (pointing up toward camera)
    %   block X-axis  -> largest spread direction on the table surface
    %   block Y-axis  -> right-hand rule from Z x X
    tableNormal = double(planeModel.Normal);          % 1x3, points toward camera
    tableNormal = tableNormal / norm(tableNormal);    % normalise
    % Ensure table normal points toward camera (+Z direction)
    if tableNormal(3) < 0
        tableNormal = -tableNormal;
    end

    figure;
    pcshow(redCloud);
    hold on;
    title('Pose Verification (PCA)');

    for k = 1:numBlocksExpected

        idx        = find(labels == sortedIdx(k));
        blockCloud = select(redCloud, idx);

        pts = blockCloud.Location;
        pts = reshape(pts, [], 3);
        pts = pts(~any(isnan(pts), 2), :);

        %% Position (centroid)
        position = mean(pts, 1);

        %% Orientation via PCA
        ptsCentered = pts - position;
        [coeff, ~, latent] = pca(ptsCentered);
        % coeff columns: [PC1=most spread, PC2=mid spread, PC3=least spread]
        % For a flat block on a table:
        %   PC1 = long axis in plane
        %   PC2 = short axis in plane
        %   PC3 = thickness/normal axis (should align with table normal)

        R = coeff;   % columns are X, Y, Z axes of block frame

        %% Fix all three PCA axis signs for global consistency

        % 1) Z axis = PC3 (normal to block face) -> align with table normal
        if dot(R(:,3), tableNormal') < 0
            R(:,3) = -R(:,3);
        end

        % 2) X axis = PC1 (longest dimension) -> enforce consistent direction
        %    Use world +X as tiebreaker: if X axis points in -X world direction, flip it
        if R(1,1) < 0
            R(:,1) = -R(:,1);
        end

        % 3) Y axis = make right-handed: Y = Z cross X
        %    This fully determines Y and eliminates the last ambiguity
        R(:,2) = cross(R(:,3), R(:,1));
        R(:,2) = R(:,2) / norm(R(:,2));   % renormalise after cross product

        % Sanity check: det should be +1 for proper rotation matrix
        if abs(det(R) - 1) > 1e-6
            warning('Block %d: R is not a proper rotation matrix (det=%.4f)', k, det(R));
        end

        eul     = rotm2eul(R);
        eul_deg = rad2deg(eul);

        T          = eye(4);
        T(1:3,1:3) = R;
        T(1:3,4)   = position';

        %% Plot centroid & orientation axes
        plot3(position(1), position(2), position(3), ...
              'o', 'MarkerSize', 10, ...
              'MarkerEdgeColor', 'y', 'MarkerFaceColor', 'y');

        scale = 0.05;
        quiver3(position(1),position(2),position(3), scale*R(1,1),scale*R(2,1),scale*R(3,1),'r','LineWidth',2,'DisplayName',sprintf('Block%d X',k));
        quiver3(position(1),position(2),position(3), scale*R(1,2),scale*R(2,2),scale*R(3,2),'g','LineWidth',2,'DisplayName',sprintf('Block%d Y',k));
        quiver3(position(1),position(2),position(3), scale*R(1,3),scale*R(2,3),scale*R(3,3),'b','LineWidth',2,'DisplayName',sprintf('Block%d Z',k));

        %% Print results
        fprintf('\n============================\n');
        fprintf('Block %d\n', k);
        fprintf('============================\n');
        fprintf('Position (meters):\n');
        fprintf('  x = %.4f\n', position(1));
        fprintf('  y = %.4f\n', position(2));
        fprintf('  z = %.4f\n\n', position(3));
        fprintf('Variance explained by each axis (%%): %.1f | %.1f | %.1f\n', ...
            100*latent(1)/sum(latent), 100*latent(2)/sum(latent), 100*latent(3)/sum(latent));
        fprintf('Rotation Matrix (R):\n'); disp(R);
        fprintf('Euler Angles ZYX (degrees):\n');
        fprintf('  Yaw   = %.2f\n', eul_deg(1));
        fprintf('  Pitch = %.2f\n', eul_deg(2));
        fprintf('  Roll  = %.2f\n\n', eul_deg(3));
        fprintf('Homogeneous Transform (T):\n'); disp(T);

    end

    legend('show');
    hold off;

end