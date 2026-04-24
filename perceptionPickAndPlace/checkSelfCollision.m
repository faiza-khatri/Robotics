function collisionDetected = checkSelfCollision(robot, q_init, q_final, numSamples)
    if nargin < 4
        numSamples = 100;  % default number of samples
    end

    if isstruct(robot) && isfield(robot, 'model')
        rbtModel = robot.model;
    else
        rbtModel = robot;
    end

    collisionDetected = false;

   
    t_values = linspace(0, 1, numSamples + 2);
    t_values = t_values(2:end-1);  % exclude endpoints -> open interval

    for i = 1:length(t_values)
        t = t_values(i);

        % interpolate between initial and final configuration
        q_interp = (1 - t) * q_init + t * q_final;

        
        [isColliding, separationDist] = checkCollision(rbtModel, q_interp, 'Exhaustive', 'on', 'SkippedSelfCollisions', 'parent');
        if any(isColliding, 'all')
            collisionDetected = true;
            fprintf('Self-collision detected at t = %.3f (sample %d/%d)\n', t, i, numSamples);
            return;
        end
       
    end

    % if ~collisionDetected
    %     disp('No self-collision detected along path.');
    % end
end
