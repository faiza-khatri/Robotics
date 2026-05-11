% Verification Script
clear; clc;

% Define test points [x, y, z, phi]
test_points = [
    20.0, 0, 15.0, 0;          % Point 1: Reaching forward, horizontal gripper
    15.0, 15.0, 10.0, deg2rad(-45); % Point 2: Diagonal, angled gripper
    10.0, 0, 25.0, deg2rad(90)     % Point 3: High reach, vertical gripper
];

for p = 1:size(test_points, 1) %[output:group:8a70a9e5]
    target = test_points(p, :);
    solutions = findJointAngles(target(1), target(2), target(3), target(4));
    
    fprintf('Testing Point %d: [%.2f, %.2f, %.2f] with Phi: %.2f rad\n', ... %[output:1f121de8] %[output:52b5d28a] %[output:1ec261cc]
            p, target(1), target(2), target(3), target(4)); %[output:1f121de8] %[output:52b5d28a] %[output:1ec261cc]
    fprintf('Number of solutions found: %d\n', size(solutions, 1)); %[output:00bfb358] %[output:95aceadb] %[output:8bc00aa0]
    
    for s = 1:size(solutions, 1)
        q = solutions(s, :);
        
        % DO NOT use servo2dh here if your IK was derived using DH frames
        [xf, yf, zf, ~, phif] = pincherFK(q); 
        
        % Calculate pure Cartesian distance error
        pos_error = sqrt((target(1)-xf)^2 + (target(2)-yf)^2 + (target(3)-zf)^2);
        
        % Check if phi matches (allowing for 180-degree flips)
        phi_error = abs(cos(target(4)) - cos(phif));
        
        fprintf('Sol %d -> Pos Error: %.4f cm, Orientation Match: %.4f\n', s, pos_error, phi_error); %[output:646070fb] %[output:95449804] %[output:9914dd86]
    end
    fprintf('---------------------------------------\n'); %[output:516bbbeb] %[output:6b1b84ab] %[output:09125709]
end %[output:group:8a70a9e5]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"inline","rightPanelPercent":43.6}
%---
%[output:1f121de8]
%   data: {"dataType":"text","outputData":{"text":"Testing Point 1: [20.00, 0.00, 15.00] with Phi: 0.00 rad\n","truncated":false}}
%---
%[output:00bfb358]
%   data: {"dataType":"text","outputData":{"text":"Number of solutions found: 2\n","truncated":false}}
%---
%[output:646070fb]
%   data: {"dataType":"text","outputData":{"text":"Sol 1 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\nSol 2 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\n","truncated":false}}
%---
%[output:516bbbeb]
%   data: {"dataType":"text","outputData":{"text":"---------------------------------------\n","truncated":false}}
%---
%[output:52b5d28a]
%   data: {"dataType":"text","outputData":{"text":"Testing Point 2: [15.00, 15.00, 10.00] with Phi: -0.79 rad\n","truncated":false}}
%---
%[output:1ec261cc]
%   data: {"dataType":"text","outputData":{"text":"Testing Point 3: [10.00, 0.00, 25.00] with Phi: 1.57 rad\n","truncated":false}}
%---
%[output:95aceadb]
%   data: {"dataType":"text","outputData":{"text":"Number of solutions found: 2\n","truncated":false}}
%---
%[output:6b1b84ab]
%   data: {"dataType":"text","outputData":{"text":"---------------------------------------\n","truncated":false}}
%---
%[output:95449804]
%   data: {"dataType":"text","outputData":{"text":"Sol 1 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\nSol 2 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\n","truncated":false}}
%---
%[output:8bc00aa0]
%   data: {"dataType":"text","outputData":{"text":"Number of solutions found: 4\n","truncated":false}}
%---
%[output:9914dd86]
%   data: {"dataType":"text","outputData":{"text":"Sol 1 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\nSol 2 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\nSol 3 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\nSol 4 -> Pos Error: 0.0000 cm, Orientation Match: 0.0000\n","truncated":false}}
%---
%[output:09125709]
%   data: {"dataType":"text","outputData":{"text":"---------------------------------------\n","truncated":false}}
%---
