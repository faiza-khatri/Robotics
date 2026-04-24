T_co = cameraToObject();
T_wo = robotToObject(T_co);
size(T_wo)
for i=1:2
    fprintf("Block %d: ", i )
    x =  T_wo(i, 1, 4); y = T_wo(i, 2, 4);
    fprintf("%f %f -0.068 4.71 %f %f -0.068 4.71\n",x, y, x, y);

end