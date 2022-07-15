function options = readyTraining1(options, mpc_first)

    options.lookupBreakpoints = 1:7;
    options.trialLength = 7;
    if mpc_first
        options.lookupValues = [ones(1, 6), 0];
    else
        options.lookupValues = [zeros(1, 7)];
    end

end