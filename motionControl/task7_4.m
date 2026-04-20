function pickAndPlace(robot)
    while true
        disp('System idle. Enter pick and place locations or q to quit.');
        
        user = input('Enter [x1 y1 z1 phi1 x2 y2 z2 phi2] or q: ', 's');
        if strcmp(user, 'q'), break; end
        
        vals = str2num(user);
        if numel(vals) ~= 8
            disp('Invalid input. Try again.');
            continue;
        end
        
        x1=vals(1); y1=vals(2); z1=vals(3); phi1=vals(4);
        x2=vals(5); y2=vals(6); z2=vals(7); phi2=vals(8);
        
        % execute the pipeline
        success = executePipeline(robot, x1,y1,z1,phi1, x2,y2,z2,phi2);
        
        if success
            disp('Task completed successfully.');
        else
            disp('Task failed. Returning to idle.');
        end
    end
end

function success = executePipeline(robot, x1,y1,z1,phi1, x2,y2,z2,phi2)
    if isstruct(robot) && isfield(robot, 'model')
        rbtModel = robot.model;
        arb = robot.hw
    else
        rbtModel = robot;
    end
    success = false;
    HOVER_HEIGHT = 0.05;  % how high above target to hover before descending
    
    %% phase 1 - home/safe pose
    disp('Phase 1: Moving to home pose...');
    home_q = [0; 0; 0; 0];

    % arb.setpos(1:4,home_q,[100 100 100 100])
    
    moveToConfig(arb, 12, home_q, 80)  % 80 steps = slow
     
    pause(1.5);
    % openGripper
    arb.setpos(5, 0, 100);
    pause(1.5);
    
    %% phase 2 - pre-grasp hover (above pick location)
    % currentConfig = arb.getpos();
    disp('Phase 2: Moving to pre-grasp hover...');
    test = [x1 y1 z1 + HOVER_HEIGHT]
    hover_q = findSolution(x1, y1, z1 + HOVER_HEIGHT, phi1, home_q, robot)
    if isempty(hover_q)
        disp('Pre-grasp hover unreachable'); return;
    end
    % arb.setpos(1:4,hover_q,[100 100 100 100])
    moveToConfig(arb, 0, hover_q, 80);
    pause(1.5);
    
    %% phase 3 - descend to grasp pose
    disp('Phase 3: Descending to grasp pose...');
    % currentConfig = arb.getpos();
    grasp_q = findSolution(x1, y1, z1, phi1, hover_q, robot);
    if isempty(grasp_q)
        disp('Grasp pose unreachable'); return;
    end
    moveToConfig(arb, 0, grasp_q, 80);
    pause(1.5);
    
    %% phase 4 - grasp
    disp('Phase 4: Grasping...');
    % currentConfig = arb.getpos();
    arb.setpos(5, pi/3.25, 100);
    pause(1.5);  % wait for gripper to close fully
    
    %% phase 5 - lift back to hover height
    disp('Phase 5: Lifting object...');
    % currentConfig = arb.getpos();
    if checkSelfCollision(robot, grasp_q, hover_q, 200)
        disp('Collision detected on lift'); return;
    end
    moveToConfig(arb,pi/3.25, hover_q, 80);
    pause(1.5);
    
    %% phase 6 - transit to above place location
    disp('Phase 6: Moving to place hover...');
    % currentConfig = arb.getpos();
    place_hover_q = findSolution(x2, y2, z2 + HOVER_HEIGHT, phi2, hover_q, robot);
    if isempty(place_hover_q)
        disp('Place hover unreachable'); return;
    end

    moveToConfig(arb, pi/3.25,place_hover_q, 80);
    pause(1.5);
    
    %% phase 7 - descend to place pose
    disp('Phase 7: Descending to place pose...');
    % currentConfig = arb.getpos();
    place_q = findSolution(x2, y2, z2, phi2, place_hover_q, robot);
    if isempty(place_q)
        disp('Place pose unreachable'); return;
    end

    moveToConfig(arb, pi/3.25, place_q, 80);
    pause(2.2);
    
    %% phase 8 - release
    disp('Phase 8: Releasing object...');
    arb.setpos(5, 0, 100);
    pause(1.5);
    
    %% phase 9 - retreat back to hover
    disp('Phase 9: Retreating...');
    if checkSelfCollision(robot, place_q, place_hover_q)
        disp('Collision on place hover'); return;
    end
    moveToConfig(arb, 0, place_hover_q, 80);
    pause(1.5);
    
    % %% phase 10 - verify placement (FK check)
    % disp('Phase 10: Verifying placement...');
    % current_q = arb.getpos();
    % [xv, yv, zv, ~] = pincherFK(current_q);
    % target_hover = [x2, y2, z2 + HOVER_HEIGHT];
    % actual_pos   = [double(xv), double(yv), double(zv)];
    % err = norm(actual_pos - target_hover);
    % fprintf('Placement verification error: %.4f m\n', err);
    
    % if err < 0.015  % 15mm tolerance
    %     success = true;
    % else
    %     disp('Placement error too large.');
    % end
    
    % return home
    moveToConfig(arb, 0, home_q, 80);
    success = true;
end

function moveToConfig(arb,gripperPos, q_final, nSteps)
    nSteps = 40;
    % q_init_full = safeGetPos(arb);
    % 
    % if isempty(q_init_full) || numel(q_init_full) < 5
    %     q_init_full = zeros(5, 1);
    % end
    % 
    % q_init  = q_init_full(1:4);
    % q_init  = q_init(:);    % force 4x1 column
    % q_final = q_final(:);   % force 4x1 column
    % gripper = q_init_full(5);
    % 
    % for i = 1:nSteps
    %     t = i / nSteps;
    %     q_interp = (1-t)*q_init + t*q_final;
    %     arb.setpos([q_interp; gripper], [100 100 100 100 100]);
    %     pause(0.02);
    % end
    speed = 70;
    
    arb.setpos([q_final; gripperPos], [speed speed speed speed speed]);

end

function q = safeGetPos(arb)
    q = arb.getpos();
    while isempty(q) || numel(q) < 5
        pause(0.05);        % wait 50ms and retry
        q = arb.getpos();
    end
end

arb = Arbotix('port', 'COM12', 'nservos', 5);
robotBundle.hw = arb;
robotBundle.model = getRobot();
pickAndPlace(robotBundle)

% positions tested:
% -0.15 0.001027 -0.069 4.7123 0.15 0.001027 -0.069 4.7123
% -0.201335 0.001027 -0.068 4.7123 -0.000160 -0.201338 -0.068 4.7123
 
