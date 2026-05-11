function optimal = findSolution(x,y,z,phi,currentConfig)
    solutions = findJointAngles(x, y, z, phi);
    optimal = [];
    minCost = inf;
    weights = [2.0, 1.5, 1.0, 0.5];
    currentConfig = currentConfig(1:4);

    if isempty(solutions)                                      
        warning('No IK solutions found for the target pose.');
        return;
    end

    for i = 1:size(solutions, 1)
        solution = solutions(i, :);
        fprintf('\n--Candidate %d: [%.4f, %.4f, %.4f, %.4f, %.4f]\n', i, solution(1), solution(2), solution(3), solution(4));

        if ~checkJointLimits(solution)
             fprintf('REJECTED: joint limits violated.\n');
            continue;
        end
       
        if SelfCollision(currentConfig, solution)
            fprintf('REJECTED: collision detected on path.\n');
            continue;
        end

        difference = mod((solution - currentConfig) + pi, 2*pi) - pi;
        cost = sum(weights.*abs(difference));

        if cost < minCost
            minCost = cost;
            optimal = solution;
        end
    end

    if isempty(optimal)
        warning("No optimal solution found\n");
    end
 end

%[appendix]{"version":"1.0"}
%---
