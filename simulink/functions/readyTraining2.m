function options = readyTraining2(options, mpc_first)

    options.lookupBreakpoints = 1:7;
    options.trialLength = 7;
    if mpc_first
        options.lookupValues = [zeros(1, 7)];
    else
        options.lookupValues = [0, ones(1, 6)];
    end

end