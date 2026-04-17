function robot = getRobot()
JOINT_MIN = -140;
JOINT_MAX = 140;

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
jnt1.PositionLimits = [JOINT_MIN JOINT_MAX]*pi/180;  % dynamixel ax-12a motor limit is +-150 degrees
body1.Joint = jnt1;                  % attaching joint to body
addCollision(body1, 'cylinder', [0.025, L1]);  % radius 2.5cm, height = L1
addBody(robot, body1, 'base');       % attaching body1 to the base of the robot

% joint 2 - shoulder
body2 = rigidBody('body2');          % creating rigid body for link 2
jnt2 = rigidBodyJoint('jnt2','revolute');  % revolute joint
T_01 = dh(0, pi/2, L1, 0);        % dh transform from 1 to 2
setFixedTransform(jnt2, T_01);      % setting the fixed transform between body1 and body2
jnt2.JointAxis = [0 0 1];           % rotates about its local z axis
jnt2.PositionLimits = [JOINT_MIN JOINT_MAX]*pi/180;  % same motor limits
body2.Joint = jnt2;
addCollision(body2, 'cylinder', [0.02, L2]);   % radius 2cm, height = L2
addBody(robot, body2, 'body1');     % attaching body2 to body1

% joint 3 - elbow
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
T_12 = dh(L2, 0, 0, pi/2);            % dh transform from frame 2 to 3
setFixedTransform(jnt3, T_12);
jnt3.JointAxis = [0 0 1];
jnt3.PositionLimits = [JOINT_MIN JOINT_MAX]*pi/180;
body3.Joint = jnt3;
addCollision(body3, 'cylinder', [0.02, L3]);   % radius 2cm, height = L3
addBody(robot, body3, 'body2');     % attaching body3 to body2

% joint 4 - wrist
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
T_23 = dh(L3, 0, 0, 0);            % dh transform from frame 3 to 4
setFixedTransform(jnt4, T_23);
jnt4.JointAxis = [0 0 1];
jnt4.PositionLimits = [JOINT_MIN JOINT_MAX]*pi/180;
body4.Joint = jnt4;
addCollision(body4, 'cylinder', [0.02, L4]);   % radius 2cm, height = L4
addBody(robot, body4, 'body3');     % attaching body4 to body3

% end-effector frame at the center of the gripper motor horn
ee = rigidBody('ee');               % no joint needed, just a fixed frame
T_34 = dh(L4, 0, 0, 0);            %dh transform from frame 4 to ee
setFixedTransform(ee.Joint, T_34); % fixed transform, ee doesnt move relative to body4
addCollision(ee, 'sphere', 0.02);              % sphere of radius 2cm for gripper
addBody(robot, ee, 'body4');        % attaching end-effector to body4

% printing robot details to verify the chain is correct
showdetails(robot)
% visualizing the robot in its zero configuration
q_home = [0; 0; 0; 0]; 
show(robot, q_home, 'Collisions', 'on');

end
function T = dh(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
