function generateReference(options)
% Generate a random sequence of gates, the reference according to that sequence
% and the cost weights according to that sequence

%% Read required options
nG = options.gateNumber; bL = options.blockLength; mW = options.maxWidth;
Te = options.Tend; Ti = options.Tinterval; Ts = options.Ts_mpc; 
TU = options.TUhorizon; Tp = options.Tpreview; Ta = options.Tafter;
nS = options.nSH; nU = options.nUH; nX = options.nX;

%% Set-up the random number generator
rng(options.randSeed, 'twister');

%% Generate a random sequence of gates
rawSequence = randi([1, nG], 1, bL);
gateLocations = linspace(-mW, mW, nG);
for i = 1 : bL
    gateSequence(i) = gateLocations(rawSequence(i));
end

%% Initialise arrays
simulinkRef = zeros(Te / Ts + nS + 1, nX);
Qarray = zeros(Te / Ts + nS + 1, nX);
unityRef = 99*ones(Te / Ts + nU + 1, 1);
t = 0 : Ts : Te+TU;

%% Begin generating the reference from the sequence
count = 1;
for i = Ti : Ti : Te
    % Fill only Gate locations into Unity
    logicalIndexes = ~(t < i-0.0000001 | t > i+0.0000001);
    unityRef(logicalIndexes, 1) = gateSequence(count);
    % Fill only Ref locations into Simulink
    logicalIndexes = ~(t < i-Tp | t > i+Ta);
    simulinkRef(logicalIndexes, 1) = gateSequence(count);
    % Fill only MPC On locations into Simulink
    logicalIndexes = ~(t < i-Tp | t > i);
    Qarray(logicalIndexes,1) = linspace(0.1, options.Q1, ...
        length(find(logicalIndexes)));
    Qarray(logicalIndexes,2) = linspace(1, options.Q2, ...
        length(find(logicalIndexes)));
    Qarray(logicalIndexes,3) = linspace(1, options.Q3, ...
        length(find(logicalIndexes)));
    % Increase the counter
    count = count+1;
end

%% Create structs from the reference
simulinkStruct = struct();
simulinkStruct.time = {};
Qstruct = struct();
Qstruct.time = {};
enumStruct = struct();
enumStruct.time = {};
blockCounterStruct = struct();
blockCounterStruct.time = {};

for i = 1 : Te/Ts + 1
    tempRef = simulinkRef(i : i+(nS-1), :);
    tempQ = Qarray(i : i+(nS-1), :);
    simulinkStruct.signals.values(i, :) = reshape(tempRef', 1, []);
    Qstruct.signals.values(i, :) = reshape(tempQ', 1, []);
end

simulinkStructInitial = simulinkStruct.signals.values(1,:);
QstructInitial = Qstruct.signals.values(1,:);
blockCounterStruct.signals.values = ...
    zeros(size(simulinkStruct.signals.values, 1), 1);
blockCounterStruct.signals.values(end) = 1;

%% Write strings containing the reference for Unity
% Create a format string
str_one_val = ",%.3f";
str_all_val = "%.3f";
for i = 1 : nU - 1
    str_all_val = str_all_val + str_one_val;
end

% Convert numeric array to a string
string_ref = "";
for i = 1 : Te/Ts + 1
    tempUnity = unityRef(i : (i+(nU-1)), :);
    enumStruct.signals.values(i, :) = i-1;
    string_ref(i, :) = sprintf(str_all_val, reshape(tempUnity', 1, []));
end

%% Export the files
cd("." + filesep + "mat_files");
save structs.mat simulinkStructInitial QstructInitial ...
     simulinkStruct Qstruct enumStruct blockCounterStruct;

fid = fopen('gate_locations.txt','w');
for i = 1:length(string_ref)
    fprintf(fid,'%s\n',string_ref(i));
end
fclose(fid);

cd("..");

end