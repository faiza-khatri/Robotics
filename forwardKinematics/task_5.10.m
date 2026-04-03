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
