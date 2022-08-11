function options = readyTraining1(options, group)

    options.lookupBreakpoints = 1:8;
    options.trialLength = 8;
    if group == 1
        options.lookupValues = [0, 0, ones(1, 6)];
    elseif group == 2
        options.lookupValues = [zeros(1, 8)];
    else
        disp("Wrong group number!");
    end

end