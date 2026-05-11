function robot = twinModelCollisions(a_servos) %arb
    %a_servos = arb.getpos();
    % a_servos = [0 0 0 0];
    dh_theta = servo2dh(a_servos(1:4));
    
    % Initialize the tree
    robot = rigidBodyTree("MaxNumBodies", 4, "DataFormat", "row");

    % DH Parameters (cm)
    a = [0, 10.187, 10.187, 6.62];
    alpha = [pi/2, 0, 0, 0];
    d = [14.121, 0, 0, 0];
    link_lengths = [14.121 10.187 10.187 6.62];

    for j = 1:4
        body  = rigidBody(['link' num2str(j)]);
        joint = rigidBodyJoint(['joint' num2str(j)], 'revolute');
        setFixedTransform(joint, [a(j) alpha(j) d(j) 0], 'dh');
        body.Joint = joint;

        %adding in collision cylinders
        len = link_lengths(j);
        collisionObj = collisionCylinder(1.5, len); 

        % adusting the pose of the collision cylinders
        if j == 1
            parentName = 'base';
            rotation = axang2tform([1 0 0 pi/2]);
            translation = trvec2tform([0, -len/2, 0]);
            collisionObj.Pose = translation * rotation;
        else
            parentName = ['link' num2str(j-1)];
            rotation = axang2tform([0 1 0 pi/2]); 
            translation = trvec2tform([-len/2, 0, 0]); 
            collisionObj.Pose = translation * rotation;
        end
        
        % Add the body using the parent's NAME (string), not the joint object
        addCollision(body, collisionObj);
        addBody(robot, body, parentName);
    end
     %showdetails(robot);
    figure;
    show(robot, dh_theta, 'Collisions', 'on', 'Visuals', 'off'); 
    title('Current Robot State');
    axis([-30 30 -30 30 0 40]);
    grid on;
end

%[appendix]{"version":"1.0"}
%---
