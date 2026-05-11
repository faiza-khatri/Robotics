function success = positionJaw(arb, position)
min_pos = 2.7;
max_pos = 0.0;


if position > min_pos || position < max_pos
    warning('positionJaw: position %.4f cm out of valid range [%.4f, %.4f] cm.', ...
        position, min_pos, max_pos);
    return;
end

try
    arb.setpos(5, position,50);

    fprintf('positionJaw: Reached %.4f cm successfully.\n', actual_pos);
    success = true;

catch e
    success=false;
    return;
end
end