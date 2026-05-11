function [] = twinModel() %arb
    %a = arb.getpos();
    a = [0 0 0 0 0];
    dh_theta = servo2dh(a(1:4));
    robot = rigidBodyTree("MaxNumBodies", 4,"DataFormat","row");

    a = [0, 10.187, 10.187, 6.62];
    alpha = [-pi/2, 0, 0, 0];
    d = [14.121, 0, 0, 0];

    for i = 1:4
        body  = rigidBody(['link' num2str(i)]);
        joint = rigidBodyJoint(['joint' num2str(i)], 'revolute');
        setFixedTransform(joint, [a(i) alpha(i) d(i) 0], 'dh');
        body.Joint = joint;
        
        % Determine the parent name
        if i == 1
            parentName = 'base'; % The first link attaches to the base
        else
            parentName = ['link' num2str(i-1)]; % Subsequent links attach to the previous one
        end
        
        % Add the body using the parent's NAME (string), not the joint object
        addBody(robot, body, parentName);
    end
    
    showdetails(robot); % Verify the structure
    figure;
    show(robot, dh_theta); % This displays the robot at the queried angles
    title('Current Robot State');
    axis([-30 30 -30 30 0 40]); % Adjust axes to see the movement clearly
    grid on;

end

%[appendix]{"version":"1.0"}
%---
