function validBlocks = unstack(robotBundle)
    % fprintf("Executing pipeline for color %s\n", color);
    [T_co, isCubeArr, numBlocksFoundRed] = cameraToObject('red');
    if(~numBlocksFoundRed)
            [T_co, isCubeArr, numBlocksFoundBlue] = cameraToObject('blue');
    end

    T_wo = robotToObject(T_co);
    size(T_wo);
    numBlocks = size(T_wo, 1);
    z2 = -0.06;

    while numBlocksFoundRed || numBlocksFoundBlue
        fprintf('Found red or blue still\n');
        if numBlocksFoundRed
            fprintf('Found red\n');
            x2 = -0.11; y2 = -0.1;
        elseif numBlocksFoundBlue
            x2 = -0.14; y2 = 0.1;
            fprintf('Found blue\n');
        end
        for i=1:numBlocks
            fprintf('I am executing pipeline ok\n');
            isCube = isCubeArr(i);
            
            fprintf("Block %d: ", i )
            x1 =  T_wo(i, 1, 4); y1 = T_wo(i, 2, 4); 
            % z1 = -0.071; 
            z1 = T_wo(i, 3, 4);
            
                radial = [x1; y1; 0];
            radial = radial / norm(radial);
        
            % Extract block X-axis in world frame (column 1 of rotation matrix)
            R_world = squeeze(T_wo(i, 1:3, 1:3));
            block_X = R_world(:, 1);   % X-axis 
            block_X(3) = 0;            % flatten to XY plane
            block_X = block_X / norm(block_X);
            
            % Radial unit vector from robot base to block
            radial = [x1; y1; 0];
            radial = radial / norm(radial);
            
            alignment = abs(dot(block_X, radial));
            fprintf(' alignment of X with radial = %.4f\n', alignment);
        
            ALIGN_THRESHOLD = 0.89;   % cos(45 deg) — tune this
    
            if ~isCube && alignment < ALIGN_THRESHOLD
                fprintf('Block %d UNGRASPABLE: X-axis not along radius (alignment=%.2f)\n', i, alignment);
                continue;
            end
    
            if(isCube) fprintf("Pick and place for cube.\n");
            else fprintf("Pick and place for rectangular.\n"); end
    
            fprintf(' block pos:    x=%.3f  y=%.3f, z=%.3f\n', x1, y1, z1);
     
            fprintf(' X alignment=%.4f  Y alignment=%.4f\n', ...
                    abs(dot(block_X, radial)), ...
                    abs(dot(R_world(1:2,2)/norm(R_world(1:2,2)), radial(1:2))));
        
            phi = 4.71; 
            executePipeline(robotBundle, x1,y1,z1,phi, x2,y2,z2,4.71);
            if numBlocksFoundRed
                y2 = y2 + 0.06; 
            elseif numBlocksFoundBlue 
                y2 = y2 - 0.06;
            end
        end

        [T_co, isCubeArr, numBlocksFoundRed] = cameraToObject('red');
        if(~numBlocksFoundRed)
            [T_co, isCubeArr, numBlocksFoundBlue] = cameraToObject('blue');
        end
        T_wo = robotToObject(T_co);
        size(T_wo);
        numBlocks = size(T_wo, 1);

    end
    

    
    
    
end


arb = Arbotix('port', 'COM12', 'nservos', 5);
robotBundle.hw = arb;
robotBundle.model = getRobot();
unstack(robotBundle);
% coloredPerceivedPickAndPlace(robotBundle, 'red', -0.14, 0.05, -0.11, 0.1);
