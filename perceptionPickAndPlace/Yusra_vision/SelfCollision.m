function isColliding = SelfCollision(initial_angles, final_angles)
    isColliding = false;
    samples = 50;
    robot = twinModelCollisions(initial_angles);

    table = collisionBox(25, 25, 1);
    table.Pose = trvec2tform([0, 0, -1]);

    for i = 1:samples
        fraction = i/samples;
        current_config = initial_angles + (fraction * (final_angles - initial_angles));
        isCollision = checkCollision(robot, servo2dh(current_config), 'SkippedSelfCollisions', 'adjacent');


        if any(isCollision) || any(checkCollision(robot, servo2dh(current_config), {table}))
            isColliding = true;
            fprintf('WARNING: Self-collision detected at path step %d!\n', i);
            fprintf('Dangerous Angles: [%.2f, %.2f, %.2f, %.2f]\n', current_config);
            break;
        end
    end

    if ~isColliding
        disp('Path is clear! No self-collisions detected.');
    end
end

%[appendix]{"version":"1.0"}
%---
