function options = readyTraining2(options, group)

    options.lookupBreakpoints = 1:7;
    options.trialLength = 7;
    if group == 1
        options.lookupValues = [zeros(1, 7)];
    elseif group == 2
        options.lookupValues = [0, ones(1, 6)];
    else
        disp("Wrong group number!");
    end

end