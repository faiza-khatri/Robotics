T_co = cameraToObject();
T_wo = robotToObject(T_co);
size(T_wo);
arb = Arbotix('port', 'COM13', 'nservos', 5);
robotBundle.hw = arb;
robotBundle.model = getRobot();
z2 = -0.071;
for i=1:3
    fprintf("Block %d: ", i )
    x1 =  T_wo(i, 1, 4); y1 = T_wo(i, 2, 4); z1 = -0.071; phi = 4.71; 
    x2 = 0.1; y2= 0.05;
    % fprintf("%f %f -0.075 4.71 %f %f -0.075 4.71\n",x, y, x, y);
    executePipeline(robotBundle, x1,y1,z,phi, x2,y2,z2,phi);
    z2 = z2 + 0.03;
end
