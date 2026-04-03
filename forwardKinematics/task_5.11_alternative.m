% task 5.11 alternative - using generateRobotWorkspace function

% generating the workspace automatically using matlab's built in function
% it uses the robot model from task 5.9 which already has joint limits set
ws = generateRobotWorkspace(robot);

%% plot 1 - 3d isometric view
figure(1);
plot(ws);                    
title('Reachable Workspace - Isometric View');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis equal; grid on;
view(45, 30);                % isometric viewing angle

%% plot 2 - top view (xy projection)
figure(2);
plot(ws);
title('Reachable Workspace - Top View (XY Plane)');
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;
view(0, 90);                 % looking straight down for top view

% computing maximum horizontal reach from workspace points
pts = ws.Points;             % getting all the reachable positions
horiz_reach = max(sqrt(pts(:,1).^2 + pts(:,2).^2));
fprintf('Maximum horizontal reach: %.4f m \n', horiz_reach);
