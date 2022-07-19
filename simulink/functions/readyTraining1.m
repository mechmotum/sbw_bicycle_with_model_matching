function options = readyTraining1(options, group)

    options.lookupBreakpoints = 1:7;
    options.trialLength = 7;
    if group == 1
        options.lookupValues = [ones(1, 6), 0];
    elseif group == 2
        options.lookupValues = [zeros(1, 7)];
    else
        disp("Wrong group number!");
    end

end