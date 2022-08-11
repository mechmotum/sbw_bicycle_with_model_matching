function options = readyTraining2(options, group)

    options.lookupBreakpoints = 1:8;
    options.trialLength = 8;
    if group == 1
        options.lookupValues = [zeros(1, 8)];
    elseif group == 2
        options.lookupValues = [0, 0, ones(1, 6)];
    else
        disp("Wrong group number!");
    end

end