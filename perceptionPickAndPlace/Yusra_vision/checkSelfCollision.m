function isColliding =  checkSelfCollision(initialJointAngles, finalJointAngles)
    numSamples = 100; 
    isColliding = false;

    robot = twinModelCollisions();
    t_full = linspace(0, 1, numSamples + 2);
    t_inter = t_full(2:end-1);

    for i = 1:length(t_inter)
        t = t_inter(i);
        
        % Linear interpolation in servo space
        q_servo = (1 - t) * initialJointAngles + t * finalJointAngles;
        
        % Convert to DH space to match your digital twin's frames
        q_dh = servo2dh(q_servo);
        
        % checkCollision returns true if the robot hits itself or an obstacle
        if checkCollision(robot, q_dh)
            isColliding = true;
            fprintf('Self-collision detected at intermediate step %d (t=%.2f)\n', i, t);
            return; % Exit immediately once a collision is found
        end
    end
end

%[appendix]{"version":"1.0"}
%---
