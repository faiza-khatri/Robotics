function errorCode = set_optimal(arb,x,y,z)
    phi = -pi/2;
    optimal = findSolution(x,y,z,phi, arb.getpos());
    if isempty(optimal)
        errorCode = 1;
        return
    end
    errorCode = 0;
    arb.setpos(2,optimal(2),50);
    arb.setpos(1,optimal(1),50);
    arb.setpos(3,optimal(3),50);
    arb.setpos(4,optimal(4),50);
end

