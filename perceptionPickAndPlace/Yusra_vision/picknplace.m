%pre motion pose:
arb.setpos(2, pi/2, 50);
arb.setpos(1, 0, 50);
arb.setpos(3, 0, 50);
arb.setpos(4, 0, 50);
pause(2);

%pick location
pick_x = -19.5;
pick_y = 0;

%place location
place_x = 19.5;
place_y = 0;

%finding IK solutions for pick location

%pre-grasp pose
% z = 10;
z = 12.5;
set_optimal(arb, pick_x, pick_y, z);
pause(4);
positionJaw(arb, 0); %end-effector completely open
pause(2);


%grasp pose
% z = 6;
z = 8.5;
set_optimal(arb, pick_x, pick_y, z);
pause(4);
positionJaw(arb, 1.15); %end-effector grasping block
pause(3);  

% pre-place pose
% z = 10;
z = 12.5;
set_optimal(arb, place_x, place_y, z);
pause(7);

% place pose
% z = 6;
z = 8.5;
set_optimal(arb, place_x, place_y, z);
pause(3)
positionJaw(arb, 0); %end-effector open enough to release the block
pause(2);

% pre-place pose
% z = 10;
z = 12.5;
set_optimal(arb, place_x, place_y, z);
pause(3)

%verifying accuracy of placement
[x,y,z,~,~] = pincherFK(arb.getpos());
pos_error = sqrt((x - place_x)^2 + (y - place_y)^2);

if pos_error < 0.5
    fprintf('Placement verified. XY error: %.4f cm\n', pos_error);
    success = true;
else
    fprintf('WARNING: Placement error %.4f cm exceeds tolerance %.4f cm\n', ...
        pos_error, 0.5);
    success = false;
end


%going back to home position
arb.setpos(1, 0,    50);
arb.setpos(2, pi/2, 50);
arb.setpos(3, 0,    50);
arb.setpos(4, 0, 50);
pause(2);

