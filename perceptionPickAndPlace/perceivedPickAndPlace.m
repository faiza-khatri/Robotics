function coloredPerceivedPickAndPlace(robotBundle, color, x21, y21, x22, y22)
    T_co = cameraToObject(color);
    T_wo = robotToObject(T_co);
    size(T_wo);
    numBlocks = size(T_wo, 1);
    
    
    z2 = -0.068;
    
    
    for i=1:numBlocks
       
        fprintf("Block %d: ", i )
        x1 =  T_wo(i, 1, 4); y1 = T_wo(i, 2, 4); z1 = -0.071; 
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
    
        ALIGN_THRESHOLD = 0.85;   % cos(32 deg) — tune this
    
        if alignment < ALIGN_THRESHOLD
            fprintf('Block %d UNGRASPABLE: X-axis not along radius (alignment=%.2f)\n', i, alignment);
            continue;
        end
    
        fprintf(' block pos:    x=%.3f  y=%.3f\n', x1, y1);
        fprintf(' radial dir:   x=%.3f  y=%.3f\n', radial(1), radial(2));
        fprintf(' block X-axis: x=%.3f  y=%.3f\n', block_X(1), block_X(2));
        fprintf(' block Y-axis: x=%.3f  y=%.3f\n', R_world(1,2), R_world(2,2));
        fprintf(' X alignment=%.4f  Y alignment=%.4f\n', ...
                abs(dot(block_X, radial)), ...
                abs(dot(R_world(1:2,2)/norm(R_world(1:2,2)), radial(1:2))));
    
        phi = 4.71; 
        x2 = x21; y2= y21;
        % fprintf("%f %f -0.075 4.71 %f %f -0.075 4.71\n",x, y, x, y);
        if (mod(i,2)==0) 
                y2 = y22;
                x2 = x22;
                
        end
        if (mod(i, 2)~=0 && i~=1)
            z2 = z2 + 0.027;
        end
            executePipeline(robotBundle, x1,y1,z1,phi, x2,y2,z2,4.71);
            % fprintf('z2 = %d at block: %d\n', z2,i);
    
          
      
    end
end

arb = Arbotix('port', 'COM12', 'nservos', 5);
robotBundle.hw = arb;
robotBundle.model = getRobot();
coloredPerceivedPickAndPlace(robotBundle, 'green', -0.11, -0.1, -0.13, -0.05);
coloredPerceivedPickAndPlace(robotBundle, 'red', -0.14, 0.05, -0.11, 0.1);
