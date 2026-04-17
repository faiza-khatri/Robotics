function optSolution = findSolution(x, y, z, phi, currentConfig, robot)
    % x, y, z - desired position
    % phi - desired orientation
    % current config

    % returns
    % [theta1, theta2, theta3, theta4] - optimal realizable inverse
    % kinematics solution

    % all angles should be within -150, 150 but we have kept it -140 140 for safe operation.
    % collision free path

    % motor limits
    JOINT_MIN = -140 * pi/180; % -5*pi/6
    JOINT_MAX =  140 * pi/180; %  5*pi/6

    allDH = findJointAngles(x, y, z, phi);
    b = [3; 3; 1; 1];          % b1 b2 b3 b4
    if isempty(allDH)
        warning('findSolution: target is out of reach – no IK solution.');
        optSolution = [];
        return;
    end

    % accomodating dh and servo difference in home config
    servo_offset = [0; 0; 0; 0];   % column vector
    
    validServo   = [];   % rows: valid servo-angle solutions
    validDH      = [];   % corresponding DH solutions (needed for FK / cost)

    for i = 1:size(allDH, 1)
        tDH    = allDH(i, :)';              % 4x1 DH angles
        tServo = tDH + servo_offset;         % 4x1 servo angles
    
        % Wrap to [-pi, pi] before limit check
        tServo = wrapToPi(tServo);
    
        if checkJointLimits(tServo, JOINT_MIN, JOINT_MAX)
            validServo(end+1, :) = tServo';  %#ok<AGROW>
            validDH(end+1, :)    = tDH';     %#ok<AGROW>
        end
    end

    if isempty(validServo)
        warning('findSolution: no solution is within joint limits.');
        optSolution = [];
        return;
    end

    q_initDH = currentConfig - servo_offset;   % current config in DH space
    
    admissibleServo = [];
    admissibleDH    = [];
    
    for i = 1:size(validDH, 1)
        q_finalDH = validDH(i, :)';
    
        collides = checkSelfCollision(robot, q_initDH, q_finalDH, 100);
    
        if ~collides
            admissibleServo(end+1, :) = validServo(i, :);  %#ok<AGROW>
            admissibleDH(end+1, :)    = validDH(i, :);     %#ok<AGROW>
        end
    end
    
    if isempty(admissibleServo)
        warning('findSolution: all reachable solutions involve a collision.');
        optSolution = [];
        return;
    end

    bestCost = inf;
    bestIdx  = 1;
    
    for i = 1:size(admissibleServo, 1)
        delta = admissibleServo(i, :)' - currentConfig;   % 4x1
        % wrap each delta to [-pi, pi]
        delta = mod(delta + pi, 2*pi) - pi;
        cost  = b' * abs(delta);
    
        if cost < bestCost
            bestCost = cost;
            bestIdx  = i;
        end
    end

    optSolution = admissibleDH(bestIdx, :)';   % 4x1 column vector
    
    fprintf('findSolution: optimal DH solution [deg]: [%.1f, %.1f, %.1f, %.1f]\n', ...
        optSolution' * 180/pi);
    fprintf('              cost = %.4f rad\n', bestCost);
    end


function a = wrapToPi(a)
    a = mod(a + pi, 2*pi) - pi;
end
% arb = Arbotix('port', 'COM5', 'nservos', 5);
% 
% function q = getCurrentPose(arb)
%     q = arb.getpos();
% end

function ok = checkJointLimits(tServo, jointMin, jointMax)
    ok = all(tServo >= jointMin) && all(tServo <= jointMax);
end
