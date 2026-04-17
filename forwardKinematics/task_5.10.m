function T = dh(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

% task 5.9
% creating a digital twin of the phantomx pincher 

% setting up the robot as a rigid body tree, using column vectors for joint angles
robot = rigidBodyTree('DataFormat','column');

% link lengths in meters
L1=0.045;  % base to shoulder height
L2=0.1035; % upper arm length
L3=0.1035; % forearm length
L4=0.11;   % wrist to gripper horn length

%setFixedTransform describes the home geometry so theta is 0

% joint 1 - base rotation
body1 = rigidBody('body1');          % creating rigid body for link 1
jnt1 = rigidBodyJoint('jnt1','revolute');  % revolute because it rotates
setFixedTransform(jnt1, eye(4));     % base frame to joint 1 is identity (no transform needed)
jnt1.JointAxis = [0 0 1];           % joint 1 rotates about z axis (base spins left/right)
jnt1.PositionLimits = [-150 150]*pi/180;  % dynamixel ax-12a motor limit is +-150 degrees
body1.Joint = jnt1;                  % attaching joint to body
addBody(robot, body1, 'base');       % attaching body1 to the base of the robot

% joint 2 - shoulder
body2 = rigidBody('body2');          % creating rigid body for link 2
jnt2 = rigidBodyJoint('jnt2','revolute');  % revolute joint
T_01 = dh(0, pi/2, L1, 0);        % dh transform from 1 to 2
setFixedTransform(jnt2, T_01);      % setting the fixed transform between body1 and body2
jnt2.JointAxis = [0 0 1];           % rotates about its local z axis
jnt2.PositionLimits = [-150 150]*pi/180;  % same motor limits
body2.Joint = jnt2;
addBody(robot, body2, 'body1');     % attaching body2 to body1

% joint 3 - elbow
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
T_12 = dh(L2, 0, 0, pi/2);            % dh transform from frame 2 to 3
setFixedTransform(jnt3, T_12);
jnt3.JointAxis = [0 0 1];
jnt3.PositionLimits = [-150 150]*pi/180;
body3.Joint = jnt3;
addBody(robot, body3, 'body2');     % attaching body3 to body2

% joint 4 - wrist
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
T_23 = dh(L3, 0, 0, 0);            % dh transform from frame 3 to 4
setFixedTransform(jnt4, T_23);
jnt4.JointAxis = [0 0 1];
jnt4.PositionLimits = [-150 150]*pi/180;
body4.Joint = jnt4;
addBody(robot, body4, 'body3');     % attaching body4 to body3

% end-effector frame at the center of the gripper motor horn
ee = rigidBody('ee');               % no joint needed, just a fixed frame
T_34 = dh(L4, 0, 0, 0);            %dh transform from frame 4 to ee
setFixedTransform(ee.Joint, T_34); % fixed transform, ee doesnt move relative to body4
addBody(robot, ee, 'body4');        % attaching end-effector to body4

% printing robot details to verify the chain is correct
showdetails(robot)
% visualizing the robot in its zero configuration
q_home = [0; 0; 0; 0]; 
show(robot, q_home);



% ------------------------------------------------------------
%task 5.6
function [x,y,z,R] = pincherFK(jointAngles)

    %jointAngles is a 1x4 vector of joint angles in radians in DH convention
    %outputs of this function are x,y,z -> end-effector position 
    % and R -> end-effector orientation

    L1 = 0.045;%meters
    L2 = 0.1035;   
    L3 = 0.1035;  
    L4 = 0.11;   

    % theta of each joint is given as argument
    theta1 = jointAngles(1);
    theta2 = jointAngles(2);
    theta3 = jointAngles(3);
    theta4 = jointAngles(4);
    
    %calculating intermediate transformations
    A01 = dh(0, pi/2, L1, theta1);
    A12 = dh(L2, 0, 0, theta2 + pi/2);
    A23 = dh(L3, 0, 0, theta3);
    A34 = dh(L4, 0, 0, theta4);

    
    %resultant transformation
    T04 = A01 * A12 * A23 * A34;

    p = T04(1:3,4); % end effector position
    R = T04(1:3,1:3); % end effector orientation

    %x = vpa(p(1), 6); y = vpa(p(2), 6); z = vpa(p(3), 6);
    x = p(1); y = p(2); z = p(3);


end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%task 5.10-checking if my pincherFK function gives same results as the digital twin

N = 1000;  % testing 1000 random arm configurations to be thorough
errors = zeros(N,1);  % making an empty array to store the position error for each test

for i = 1:N
    % generating a random joint configuration that respects the +-150 degree motor limits
    % q is a 4x1 column vector of joint angles in radians
    q = randomConfiguration(robot);

    % getting the end-effector transform from the digital twin
    % getTransform uses the rigidBodyTree model we built in task 5.9
    % T_twin is a 4x4 homogeneous transform matrix
    T_twin = getTransform(robot, q, 'ee');

    % passing the same joint angles to pincherFK function
    % q(1) to q(4) extracts each joint angle from the column vector
    dhq = [q(1); q(2); q(3); q(4)];
    
    % it returns x,y,z position and R rotation matrix of end-effector
    [x,y,z,R] = pincherFK(dhq);
    
    % putting my xyz output into a single column vector 
    p_my = [x; y; z];
    
    % extracting the position from the digital twin's 4x4 transform matrix
    p_twin  = T_twin(1:3, 4);

    % computing the distance (error) between my result and the digital twin result
    % norm() gives the euclidean distance between the two position vectors
    % if my FK is correct this should be basically 0 
    errors(i) = norm(p_my - p_twin);
end

% printing the worst case error across all 1000 tests
fprintf('Max position error: %.6f m\n', max(errors));
% printing the average error
fprintf('Mean position error: %.6f m\n', mean(errors));

% if max error is less than 1e-6 meters (0.001 mm) then its just floating point noise
% meaning my FK matches the digital twin perfectly
if max(errors) < 1e-6
    disp('FK PASSED - matches digital twin to numerical precision');
else
    % if error is large then either my DH parameters are wrong
    disp('FK FAILED - check your DH parameters or link lengths');
end
