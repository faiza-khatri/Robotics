% task 5.11 - finding the reachable workspace using monte carlo simulation
% randomly sample 50000 joint angle combinations,
%compute where the gripper ends up for each one using pincherFK,
% then plot all those gripper positions to see what space the arm can reach

N = 50000;  %  - more samples -> more accurate workspace picture

% converting 150 degrees to radians because our FK function works in radians
% this is the physical limit of each dynamixel ax-12a motor
lim = 150 * pi/180;

% generating N random angles for each joint between -150 and +150 degrees
% formula used: theta = min + (max - min) * rand(N,1)
% so -lim + 2*lim*rand gives random numbers between -lim and +lim
theta1 = -lim + (lim - (-lim))*rand(N,1);  % N random angles for base joint
theta2 = -lim + 2*lim*rand(N,1);  % N random angles for shoulder joint
theta3 = -lim + 2*lim*rand(N,1);  % N random angles for elbow joint
theta4 = -lim + 2*lim*rand(N,1);  % N random angles for wrist joint

% making an empty Nx3 matrix to store the x,y,z position for each configuration
positions = zeros(N,3);

% looping through all 50000 random configurations
for i = 1:N
    % running forward kinematics for this random joint combination
    [x,y,z,~] = pincherFK([theta1(i), theta2(i), theta3(i), theta4(i)]);
    
    % storing the gripper position for this configuration as a row in our matrix
    positions(i,:) = [x, y, z];
end

%% plot 1 - 3d isometric view of the entire workspace
figure(1);
scatter3(positions(:,1), positions(:,2), positions(:,3), ...
         1, ...                    % dot size = 1 
         positions(:,3), ...       % coloring each dot by its height (z value)
         'filled', ...             % filled dots
         'MarkerFaceAlpha', 0.1);  % making dots transparent so we can see density
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Reachable Workspace - Isometric View');
colorbar;       
axis equal;     % making sure x,y,z axes are same scale so shape isnt distorted
grid on;
view(45, 30);   % setting the viewing angle to isometric (45 degrees horizontal, 30 up)

% plot 2 - top view 
figure(2);
scatter(positions(:,1), positions(:,2), ...
        1, ...                                              % dot size
        sqrt(positions(:,1).^2 + positions(:,2).^2), ...   % coloring by horizontal distance from base
        'filled', ...
        'MarkerFaceAlpha', 0.1);
xlabel('X (m)'); ylabel('Y (m)');
title('Reachable Workspace - Top View (XY Plane)');
colorbar;       
axis equal;
grid on;

% computing the maximum horizontal reach
% horizontal distance from base = sqrt(x^2 + y^2) 
horiz_reach = max(sqrt(positions(:,1).^2 + positions(:,2).^2));
fprintf('Maximum horizontal reach: %.4f m \n', horiz_reach);

% plot 3 - convex 
% instead of showing all 50000 dots, this draws just boundary
% convhull finds the smallest shape that contains all the points
% trisurf draws that shape as a 3d surface
figure(3);
k = convhull(positions(:,1), positions(:,2), positions(:,3));  % computing the outer boundary
trisurf(k, positions(:,1), positions(:,2), positions(:,3), ... % drawing the boundary surface
        'FaceAlpha', 0.15, ...   % making surface transparent so we can see inside
        'EdgeColor', 'none');    % hiding the triangle edges for a cleaner look
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Workspace Convex Hull');
axis equal; grid on;
